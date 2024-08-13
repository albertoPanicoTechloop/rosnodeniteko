
# app.py

from flask import Flask, render_template, request, redirect, url_for, jsonify, session
import threading
import os
import sys
import rospkg
import subprocess
import rospy
import time
from robot.msg import canread
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu
from tf.transformations import euler_from_quaternion

# Ensure the script can find the templates directory regardless of how it's run
package_path = rospkg.RosPack().get_path('vinya_st4030_gui')
template_path = os.path.join(package_path, 'scripts/templates')
static_path = os.path.join(package_path, 'scripts/static')
print("Template Path: ", template_path)
print("Static Path: ", static_path)
print("Files in template path: ", os.listdir(template_path))
print("Files in static path: ", os.listdir(static_path))

app = Flask(__name__, template_folder=template_path, static_folder=static_path)
app.secret_key = 'your_secret_key'  # Needed for session management

button_state = {
    'start_disabled': False,
    'pause_disabled': False,
    'stop_disabled': False,
    'semiauto_state': False,
    'autospray_state': False,
    'record_path_state': 'RECORD PATH',  # Initial state
    'path_name_set': False,  # Initial state for path name
    'load_path_state': 'START PUBLISH', # Initial state for load path
    'tracker_only_state': False,  # Initial state for tracker only
    'border_guard_state': False,  # Initial state for border guard
    'forward_anticoll_state': False   # Initial state for anti collision
}

ecu_status = {
    'active': False,
    'last_update': time.time()
}

ecu_status_lock = threading.Lock()

# Add a global variable to store the linear speed
linear_speed = {
    'x': 0.0
}
linear_speed_lock = threading.Lock()

# Add global variable and lock for IMU data
imu_data = {
    'roll': 0.0,
    'pitch': 0.0,
    'yaw': 0.0
}
imu_data_lock = threading.Lock()

def initialize_ros_node():
    argv = rospy.myargv(argv=sys.argv)
    rospy.init_node('web_controller', anonymous=True, argv=argv)
    #rospy.set_param('/use_sim_time', False)

def cmd_vel_callback(msg):
    global linear_speed
    with linear_speed_lock:
        linear_speed['x'] = msg.linear.x*3.6

def monitor_cmd_vel_topic():
    rospy.Subscriber("/platform/cmd_vel", Twist, cmd_vel_callback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

def imu_callback(msg):
    global imu_data
    orientation_q = msg.orientation
    orientation_list = [orientation_q.x, orientation_q.y, orientation_q.z, orientation_q.w]
    (roll, pitch, yaw) = euler_from_quaternion(orientation_list)
    with imu_data_lock:
        imu_data['roll'] = roll*180/3.1416
        imu_data['pitch'] = pitch*180/3.1416
        imu_data['yaw'] = yaw*180/3.1416

def monitor_imu_topic():
    rospy.Subscriber("/multiScan/imu", Imu, imu_callback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        rate.sleep()

def canread_callback(msg):
    global ecu_status
    with ecu_status_lock:
        ecu_status['active'] = True
        ecu_status['last_update'] = time.time()

def monitor_canread_topic():
    rospy.Subscriber("/canread_msg", canread, canread_callback)
    rate = rospy.Rate(1)
    while not rospy.is_shutdown():
        with ecu_status_lock:
            if time.time() - ecu_status['last_update'] > 2:
                ecu_status['active'] = False
        rate.sleep()

def set_start_params():
    try:
        rospy.set_param('/mission_exec', True)
        rospy.set_param('/row_autonav', True)
        rospy.set_param('engageTracker', True)
        rospy.set_param('/semiAutoRowFollow', False)
        rospy.set_param('vb100c_motion_pause_cmd', False)
        print("Start parameters set successfully")
    except Exception as e:
        print(f"Failed to set start parameters: {e}")

def set_pause_params():
    try:
        rospy.set_param('vb100c_motion_pause_cmd', True)
        print("Pause parameters set successfully")
    except Exception as e:
        print(f"Failed to set pause parameters: {e}")

def set_stop_params():
    try:
        rospy.set_param('/mission_exec', False)
        rospy.set_param('/number_of_turns_', 0)
        rospy.set_param('/semiAutoRowFollow', False)
        rospy.set_param('/engageTracker', False)
        print("Stop parameters set successfully")
    except Exception as e:
        print(f"Failed to set stop parameters: {e}")

def set_des_turns(value):
    try:
        rospy.set_param('/des_turns', value)
        print(f"Set /des_turns to {value}")
    except Exception as e:
        print(f"Failed to set /des_turns: {e}")

def get_des_turns():
    try:
        return rospy.get_param('/des_turns', 0)
    except Exception as e:
        print(f"Failed to get /des_turns: {e}")
        return 0

def set_semiauto_params(pressed):
    try:
        rospy.set_param('vb100c_motion_pause_cmd', False)
        rospy.set_param('/semiAutoRowFollow', pressed)
        rospy.set_param('/mission_exec', pressed)
        rospy.set_param('/row_autonav', pressed)
        if pressed:
            print("Semiauto parameters set to pressed state")
        else:
            print("Semiauto parameters set to depressed state")
    except Exception as e:
        print(f"Failed to set semiauto parameters: {e}")

def set_autospray_params(pressed):
    try:
        rospy.set_param('/enable_sprayer', pressed)
        if pressed:
            print("Autospray parameters set to pressed state")
        else:
            print("Autospray parameters set to depressed state")
    except Exception as e:
        print(f"Failed to set autospray parameters: {e}")

def restart_autopilot():
    try:
        rospy.set_param('/row_autonav', False)
        print("Set /row_autonav to False")
        
        nodes_to_restart = [
            "/vb100c_row_redux",
            "/vb100c_row_vision",
            "/vb100c_interrow_guardian",
            "/vinyaMAPs",
            "/vb100c_path_tracker"
        ]
        
        for node in nodes_to_restart:
            subprocess.call(['rosnode', 'kill', node])
            print(f"Restarted node {node}")
        
        rospack = rospkg.RosPack()
        package_path = rospack.get_path('vinya_st4030_launch')
        launch_file_path = os.path.join(package_path, 'launch', 'DEFAULT.launch')
        
        subprocess.call(['roslaunch', launch_file_path])
        print(f"Loaded {launch_file_path}")
    except Exception as e:
        print(f"Failed to restart autopilot: {e}")

def start_can_node():
    try:
        subprocess.call(['rosrun', 'canhymach', 'canhymach_node'])
        print("CANHYMACH node started")
    except Exception as e:
        print(f"Failed to start CANHYMACH node: {e}")

def stop_can_node():
    try:
        subprocess.call(['rosnode', 'kill', '/canhymach_node'])
        print("CANHYMACH node stopped")
    except Exception as e:
        print(f"Failed to stop CANHYMACH node: {e}")

def start_recording_path():
    try:
        subprocess.Popen(['rosnode', 'kill', '/vb100c_path_publisher'])
        print("/vb100c_path_publisher node killed")
        subprocess.Popen(['rosrun', 'vb100c_path_recorder', 'vb100c_path_recorder'])
        print("vb100c_path_recorder node started")
    except Exception as e:
        print(f"Failed to start recording path: {e}")

def stop_recording_path(user_defined_name):
    try:
        subprocess.Popen(['rosnode', 'kill', '/vb100c_path_recorder'])
        print("vb100c_path_recorder node killed")
        
        rospack = rospkg.RosPack()
        src_path = os.path.join(rospack.get_path('vb100c_path_recorder'), 'rec_poses.tmp')
        dest_path = os.path.join(rospack.get_path('vinya_st4030_gui'), 'scripts/launch/PATHS', user_defined_name)
        
        if os.path.exists(src_path):
            os.rename(src_path, dest_path)
            print(f"File moved from {src_path} to {dest_path}")
        else:
            print(f"Source file {src_path} does not exist")
    except Exception as e:
        print(f"Failed to stop recording path: {e}")

def set_tracker_only(pressed):
    try:
        if pressed:
            rospy.set_param('engageTracker', True)
            rospy.set_param('vb100c_motion_pause_cmd', False)
            print("Tracker engaged")
        else:
            rospy.set_param('vb100c_motion_pause_cmd', True)
            print("Tracker disengaged")
    except Exception as e:
        print(f"Failed to set tracker state: {e}")

# Initialize ROS node
initialize_ros_node()

# Start monitoring thread
monitor_thread = threading.Thread(target=monitor_canread_topic)
monitor_thread.daemon = True
monitor_thread.start()

monitor_cmd_vel_thread = threading.Thread(target=monitor_cmd_vel_topic)
monitor_cmd_vel_thread.daemon = True
monitor_cmd_vel_thread.start()

#Start the IMU monitoring thread
monitor_imu_thread = threading.Thread(target=monitor_imu_topic)
monitor_imu_thread.daemon = True
monitor_imu_thread.start()


@app.route('/')
def index():
    des_turns = get_des_turns()
    paths = os.listdir(os.path.join(rospkg.RosPack().get_path('vinya_st4030_gui'), 'scripts/launch/PATHS'))
    return render_template('index.html', button_state=button_state, des_turns=des_turns, paths=paths)

@app.route('/start', methods=['POST'])
def start():
    set_start_params()
    button_state['start_disabled'] = True
    button_state['pause_disabled'] = False
    button_state['stop_disabled'] = False
    button_state['semiauto_state'] = False
    return redirect(url_for('index'))

@app.route('/pause', methods=['POST'])
def pause():
    set_pause_params()
    button_state['start_disabled'] = False
    return redirect(url_for('index'))

@app.route('/stop', methods=['POST'])
def stop():
    set_stop_params()
    button_state['start_disabled'] = False
    return redirect(url_for('index'))

@app.route('/semiauto', methods=['POST'])
def semiauto():
    button_state['semiauto_state'] = not button_state['semiauto_state']
    set_semiauto_params(button_state['semiauto_state'])
    return redirect(url_for('index'))

@app.route('/autospray', methods=['POST'])
def autospray():
    button_state['autospray_state'] = not button_state['autospray_state']
    set_autospray_params(button_state['autospray_state'])
    return redirect(url_for('index'))

@app.route('/restart_autopilot', methods=['POST'])
def restart_autopilot_route():
    restart_autopilot()
    return redirect(url_for('index'))

@app.route('/set_des_turns', methods=['POST'])
def set_des_turns_route():
    value = request.form.get('des_turns', type=int)
    set_des_turns(value)
    return redirect(url_for('index'))

@app.route('/canon', methods=['POST'])
def canon():
    start_can_node()
    return redirect(url_for('index'))

@app.route('/canoff', methods=['POST'])
def canoff():
    stop_can_node()
    return redirect(url_for('index'))

@app.route('/ecu_status', methods=['GET'])
def get_ecu_status():
    with ecu_status_lock:
        return jsonify(ecu_status)

@app.route('/set_path_name', methods=['POST'])
def set_path_name():
    user_defined_name = request.form.get('path_name')
    if not user_defined_name.endswith('.path'):
        user_defined_name += '.path'
    session['user_defined_name'] = user_defined_name
    button_state['path_name_set'] = True  # Path name has been set
    return redirect(url_for('index'))

@app.route('/record_path', methods=['POST'])
def record_path():
    if 'user_defined_name' not in session:
        return redirect(url_for('index'))  # or handle error
    user_defined_name = session['user_defined_name']
    if button_state['record_path_state'] == 'RECORD PATH':
        start_recording_path()
        button_state['record_path_state'] = 'STOP RECORD'
    else:
        stop_recording_path(user_defined_name)
        button_state['record_path_state'] = 'RECORD PATH'
        button_state['path_name_set'] = False  # Reset path name set state
    return redirect(url_for('index'))

@app.route('/load_path', methods=['POST'])
def load_path():
    rospack = rospkg.RosPack()
    paths_dir = os.path.join(rospack.get_path('vinya_st4030_gui'), 'scripts/launch/PATHS')
    
    if button_state['load_path_state'] == 'START PUBLISH':
        path_filename = request.form.get('path_load_location')
        if path_filename:
            pathLoadLocation = os.path.join(paths_dir, path_filename)
            try:
                subprocess.Popen(['rosnode', 'kill', '/vb100c_path_recorder'])
            except Exception as e:
                print(f"Failed to kill /vb100c_path_recorder: {e}")
            subprocess.Popen(['rosrun', 'vb100c_path_publisher', 'vb100c_path_publisher', pathLoadLocation])
            print(f"Path loaded from {pathLoadLocation}")
            button_state['load_path_state'] = 'STOP PUBLISH'
        else:
            print("Path not selected")
    else:
        try:
            subprocess.Popen(['rosnode', 'kill', '/vb100c_path_publisher'])
            print("vb100c_path_publisher node killed")
        except Exception as e:
            print(f"Failed to stop path publishing: {e}")
        button_state['load_path_state'] = 'START PUBLISH'
    return redirect(url_for('index'))

@app.route('/tracker_only', methods=['POST'])
def tracker_only():
    button_state['tracker_only_state'] = not button_state['tracker_only_state']
    set_tracker_only(button_state['tracker_only_state'])
    return redirect(url_for('index'))

@app.route('/linear_speed', methods=['GET'])
def get_linear_speed():
    with linear_speed_lock:
        return jsonify(linear_speed)
    
@app.route('/imu_data', methods=['GET'])
def get_imu_data():
    with imu_data_lock:
        return jsonify(imu_data)
    
@app.route('/border_guard', methods=['POST'])
def border_guard():
    button_state['border_guard_state'] = not button_state.get('border_guard_state', False)
    set_interrow_guardian(button_state['border_guard_state'])
    return redirect(url_for('index'))

def set_interrow_guardian(pressed):
    try:
        rospy.set_param('/interrow_guardian_up', not pressed)
        if pressed:
            print("Interrow guardian set to false")
        else:
            print("Interrow guardian set to true")
    except Exception as e:
        print(f"Failed to set interrow guardian state: {e}")

@app.route('/forward_anticoll', methods=['POST'])
def forward_anticoll():
    button_state['forward_anticoll_state'] = not button_state.get('forward_anticoll_state', False)
    set_anti_collision(button_state['forward_anticoll_state'])
    return redirect(url_for('index'))

def set_anti_collision(pressed):
    try:
        rospy.set_param('/enable_anti_collision', not pressed)
        if pressed:
            print("Anti collision set to false")
        else:
            print("Anti collision set to true")
    except Exception as e:
        print(f"Failed to set anti collision state: {e}")        


if __name__ == '__main__':
    app.run(host='192.168.59.249', port=5001, debug=False)
