import sys
import matplotlib.pyplot as plt
import numpy as np

# Get the data values from the command-line argument
data = np.array(sys.argv[1].split(','), dtype=float)

# Reshape the data into a 2D array with two columns
points = data.reshape(-1, 2)

# Create a scatter plot of the data
plt.scatter(points[:, 0], points[:, 1])

# Add axis labels and a title
plt.xlabel('X values')
plt.ylabel('Y values')
plt.title('Scatter plot example')

# Show the plot
plt.show()
