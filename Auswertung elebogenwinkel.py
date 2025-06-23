import pandas as pd
import matplotlib.pyplot as plt

# Load the data
csv_file = 'armlaengen_niko.csv'
data = pd.read_csv(csv_file, header=None, names=['RawData'])

# Debug: Check the raw data
print("Raw data preview:")
print(data.head())

# Extract time, type, and value from the raw data
data[['Time', 'Elbogenwinkel Type', 'Value']] = data['RawData'].str.extract(
    r'(\d+)\s+elbogenwinkel\s+(links|rechts):\s*([\d\.]+)'
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
