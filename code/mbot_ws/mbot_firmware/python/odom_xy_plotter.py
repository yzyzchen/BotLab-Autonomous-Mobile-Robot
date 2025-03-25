import matplotlib.pyplot as plt

# Define the path to your data file
file_path = '/home/sgari/python3_sandbox/odom_log_2.txt'

# Initialize lists to store the data
x_values, y_values = [], []

# Read the data from the file
with open(file_path, 'r') as file:
    for line in file:
        x, y, _ = map(float, line.split(','))
        x_values.append(x)
        y_values.append(y)

# Plotting
plt.figure(figsize=(10, 6))
plt.plot(y_values, x_values, linestyle='-', label='Path')
plt.title('X vs Y slow speed (~0.2m/s & pi/4 rad/s)')
plt.xlabel('Y axis')
plt.ylabel('X axis')
plt.grid(True)
plt.axis('equal')  # This makes the axes equal

# Adding start and end markers
plt.scatter(y_values[0], x_values[0], color='green', s=100, zorder=5, label='Start')
plt.scatter(y_values[-1], x_values[-1], color='red', s=100, zorder=5, label='End')

# Adding a legend
plt.legend()

plt.show()
