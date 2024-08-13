import csv

def reverse_rows_recursively(data, reversed_data=None):
    if reversed_data is None:
        reversed_data = []
        
    if len(data) == 0:
        return reversed_data
    
    reversed_data.append(data.pop())
    return reverse_rows_recursively(data, reversed_data)

def main():
    input_file = 'Transfer-1-CANDIANA-0.5m.csv'
    output_file = 'Transfer-1-CANDIANA-0.5m_reverted.csv'
    
    with open(input_file, 'r') as f:
        reader = csv.reader(f)
        header = next(reader)
        rows = list(reader)
    
    reversed_rows = reverse_rows_recursively(rows)
    
    with open(output_file, 'w', newline='') as f:
        writer = csv.writer(f)
        writer.writerow(header)
        writer.writerows(reversed_rows)

if __name__ == '__main__':
    main()

