import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load the CSV file with parsed data
file_path = "lidar_data_parsed.csv"  # Replace with your actual file path
data = pd.read_csv(file_path)

# Validate necessary columns
required_columns = ["Angle (degrees)", "Distance (mm)", "Intensity"]
if not all(col in data.columns for col in required_columns):
    raise ValueError(f"CSV file must contain these columns: {', '.join(required_columns)}")

# Adjust angles to align with the North-based orientation
data['Adjusted Angle (degrees)'] = (90 - data['Angle (degrees)']) % 360
data['Adjusted Angle (radians)'] = np.radians(data['Adjusted Angle (degrees)'])

# Calculate Cartesian coordinates (x, y) using adjusted angle and distance
data['x'] = data['Distance (mm)'] * np.cos(data['Adjusted Angle (radians)'])
data['y'] = data['Distance (mm)'] * np.sin(data['Adjusted Angle (radians)'])

# Create a 2D map visualization
plt.figure(figsize=(12, 12))
scatter = plt.scatter(
    data['x'], data['y'], c=data['Intensity'], cmap='viridis', s=10, alpha=0.7
)
plt.colorbar(scatter, label="Intensity")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.title("2D Lidar Map (North-Oriented): Angle vs Distance with Intensity")
plt.axis('equal')  # Ensure equal aspect ratio
plt.grid(True)

# Add a marker for the lidar's position
plt.scatter(0, 0, c='red', label="Lidar (0, 0)", s=100, edgecolor='black')
plt.legend()
plt.show()

