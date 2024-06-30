import numpy as np
from scipy.interpolate import griddata

class SPMLookupTable:
    def __init__(self, data):
        self.data = data
        self.points = data[:, 1:3]  # x and y coordinates (assuming they're in columns 1 and 2)
        self.values_m1 = data[:, 3]  # m1 values
        self.values_m2 = data[:, 4]  # m2 values
        self.values_m3 = data[:, 5]  # m3 values

        # Store the min and max values for x and y
        self.x_min, self.x_max = np.min(self.points[:, 0]), np.max(self.points[:, 0])
        self.y_min, self.y_max = np.min(self.points[:, 1]), np.max(self.points[:, 1])

    def lookup(self, x, y):
        # Check if the point is within the data range
        if not (self.x_min <= x <= self.x_max and self.y_min <= y <= self.y_max):
            print(f"Warning: Input point ({x}, {y}) is outside the data range.")
            return np.array([np.nan, np.nan, np.nan])

        point = np.array([[x, y]])
        m1 = griddata(self.points, self.values_m1, point, method='linear')[0]
        m2 = griddata(self.points, self.values_m2, point, method='linear')[0]
        m3 = griddata(self.points, self.values_m3, point, method='linear')[0]
        return np.array([m1, m2, m3])

# Load data
data = np.loadtxt('data.csv', delimiter=',', skiprows=1)
print("Data shape:", data.shape)
print("First few rows:")
print(data[:5])  # Print first 5 rows to verify column order

# Create lookup table
lut = SPMLookupTable(data)

# Example usage
x, y = 0.23, -0.17
result = lut.lookup(x, y)
print(f"For x={x}, y={y}: result={result}")
m1, m2, m3 = result
print(f"m1={m1:.4f}, m2={m2:.4f}, m3={m3:.4f}")

# Print the data range
print(f"X range: {lut.x_min:.2f} to {lut.x_max:.2f}")
print(f"Y range: {lut.y_min:.2f} to {lut.y_max:.2f}")


x, y = 0.1, 0.2
result = lut.lookup(x, y)
print(f"For x={x}, y={y}: result={result}")
m1, m2, m3 = result
print(f"m1={m1:.4f}, m2={m2:.4f}, m3={m3:.4f}")