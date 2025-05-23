import pandas as pd
import matplotlib.pyplot as plt

# Load the data
csv_file = 'armlaengen.csv'
data = pd.read_csv(csv_file)

# Debug: Check what columns are available
print("Available columns:", data.columns.tolist())

# Extract time, type, and value from the first column
data[['Time', 'Elbogenwinkel Type', 'Value']] = data.iloc[:, 0].str.extract(
    r'test (\d+) elbogenwinkel (links|rechts):\s*([\d\.]+|nan)'
)

# Convert to numeric
data['Time'] = pd.to_numeric(data['Time'], errors='coerce')
data['Value'] = pd.to_numeric(data['Value'], errors='coerce')

# Split into left and right
data_links = data[data['Elbogenwinkel Type'] == 'links']
data_rechts = data[data['Elbogenwinkel Type'] == 'rechts']

# Plot rechts
plt.figure(figsize=(10, 6))
plt.plot(data_rechts['Time'], data_rechts['Value'], marker='o', linestyle='-', color='r', label='Elbogenwinkel Rechts')
plt.xlabel('Time')
plt.ylabel('Elbogenwinkel')
plt.title('Elbogenwinkel Rechts')
plt.legend()
plt.grid(True)
plt.show()

# Plot links
plt.figure(figsize=(10, 6))
plt.plot(data_links['Time'], data_links['Value'], marker='o', linestyle='-', color='g', label='Elbogenwinkel Links')
plt.xlabel('Time')
plt.ylabel('Elbogenwinkel')
plt.title('Elbogenwinkel Links')
plt.legend()
plt.grid(True)
plt.show()
