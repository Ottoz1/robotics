import sys
import matplotlib.pyplot as plt
import numpy as np

# Get the x and y values of the points from the command-line argument
data = np.array(sys.argv[1].split(','), dtype=float)

# Split the data into two arrays with equal length
x_points = data[0::2]
y_points = data[1::2]

# Create a 2D array with the x and y values of the points
points = np.column_stack((x_points, y_points))

# Get the x and y values of the line segments from the command-line argument
data = np.array(sys.argv[2].split(','), dtype=float)

# Split the data into two arrays with equal length
x_lines = data[0::2]
y_lines = data[1::2]

# Create a 2D array with the x and y values of the line segments
lines = np.column_stack((x_lines, y_lines))

# Get the title from the command-line argument
title = sys.argv[3]

# Create a scatter plot of the points
plt.scatter(points[:, 0], points[:, 1])

# Create a line plot of the line segments
plt.plot(lines[:, 0], lines[:, 1], '-', color='red')

# Add axis labels and a title
plt.xlabel('X values')
plt.ylabel('Y values')
plt.title(title)

# Show the plot
plt.show()
