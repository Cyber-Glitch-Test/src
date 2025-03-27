import csv
test = 5
with open('armlaengen.csv', 'a',newline='') as f:
            writer = csv.writer(f)
            writer.writerow([f'oberarmlänge{test}'])
            writer.writerow([f'unterarmlänge'])