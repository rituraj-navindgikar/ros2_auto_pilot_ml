import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

# Load the CSV file with parsed data
file_path = "lidar_data_parsed.csv"  # Replace with your actual file path
data = pd.read_csv(file_path)

# Validate necessary columns
required_columns = [
    "Timestamp (ms)", "RPM", "Angle (degrees)", "Distance (mm)",
    "Intensity", "Velocity (mm/frame)", "Height (mm)", "Object Classification"
]
if not all(col in data.columns for col in required_columns):
    raise ValueError(f"CSV file must contain these columns: {', '.join(required_columns)}")

# Convert angle (degrees) to radians for Cartesian transformation
data['Angle (radians)'] = np.radians(data['Angle (degrees)'])

# Calculate Cartesian coordinates (x, y) using angle and distance
data['x'] = data['Distance (mm)'] * np.cos(data['Angle (radians)'])
data['y'] = data['Distance (mm)'] * np.sin(data['Angle (radians)'])

# Create a 2D map visualization
plt.figure(figsize=(12, 12))
scatter = plt.scatter(
    data['x'], data['y'], c=data['Intensity'], cmap='viridis', s=10, alpha=0.7
)
plt.colorbar(scatter, label="Intensity")
plt.xlabel("X (mm)")
plt.ylabel("Y (mm)")
plt.title("2D Lidar Map: Angle vs Distance with Intensity")
plt.axis('equal')  # Ensure equal aspect ratio
plt.grid(True)
plt.show()

