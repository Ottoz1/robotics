import numpy as np
import matplotlib.pyplot as plt

corner_points = np.array([[0, 0], [0, 1], [2, 1], [2, 0]]) * 100
plt.plot(corner_points[:, 0], corner_points[:, 1], 'bo')

# Set the seed for reproducibility
np.random.seed(0)

# Make a square from the corner points
line_segs = np.array([[0, 1], [1, 2], [2, 3], [3, 0]])  # 0-1, 1-3, 3-2, 2-0
line_mod = np.array([corner_points[line_segs[0, :]], corner_points[line_segs[1, :]], corner_points[line_segs[2, :]], corner_points[line_segs[3, :]]])
for line_seg in line_segs:
    plt.plot(corner_points[line_seg, 0], corner_points[line_seg, 1], 'b-')

# Generate random points on the line segments
num_points = 100
points = np.zeros((num_points, 2))
for i in range(num_points):
    line_seg = line_segs[np.random.randint(0, 4)]
    points[i, :] = corner_points[line_seg[0], :] + np.random.rand() * (corner_points[line_seg[1], :] - corner_points[line_seg[0], :])

# Add noise
points += 0.01 * np.random.randn(num_points, 2)

# Apply random rotation and translation
theta = np.random.rand() * np.pi/2
R = np.array([[np.cos(theta), -np.sin(theta)], [np.sin(theta), np.cos(theta)]])

# Shift the rotation center to the center of mass
mean = np.mean(points, axis=0)
points -= mean

points = np.dot(points, R)
points += mean

plt.plot(points[:, 0], points[:, 1], 'r.')

plt.axis('equal')
plt.show()


### Solve the Point-set registration problem ###
def find_dist_to_lineseg(point, line):
    # point: a 2D point
    # line: a 2D point-pair, each pair defines a line segment
    # normal: the normal vector of the line segment
    # return: the distance from the point to the line segment (can be negative)

    lx1, ly1 = line[0]
    lx2, ly2 = line[1]
    px, py = point

    # Calculate position of closest point to point on line segment.
    line_dist = ((lx2 - lx1) ** 2 + (ly2 - ly1) ** 2)
    t = ((px - lx1) * (lx2 - lx1) + (py - ly1) * (ly2 - ly1)) / line_dist

    # Clamp t to lie between 0 and 1 to ensure closest point is actually on line segment.
    t = max(0, min(t, 1))

    # Calculate coordinates of closest point on line segment.
    closest_x = lx1 + t * (lx2 - lx1)
    closest_y = ly1 + t * (ly2 - ly1)

    # Calculate squared distance between point and closest point on line segment.
    dist_sq_to_segment = (px - closest_x) ** 2 + (py - closest_y) ** 2

    # Return squared distance.
    return dist_sq_to_segment


def find_normals(line_model):
    # line_model: a list of 2D point-pairs, each pair defines a line segment
    # return: a list of normal vectors, one for each line segment

    normals = []
    for line in line_model:
        x1, y1 = line[0]
        x2, y2 = line[1]
        x3 = y1 - y2
        y3 = x2 - x1
        normal = [x3, y3]
        normal /= np.linalg.norm(normal)
        normals.append(normal)

    return normals


def cox_linefit(points, line_model, max_iter):
    # points: a list of 2D points
    # line_model: a list of 2D point-pairs, each pair defines a line segment
    # max_iter: the maximum number of iterations
    # return: dx, dy, da, the translation and rotation to be applied to the points to best fit the line model

    point_copy = points.copy()
    point_copy = np.hstack((point_copy, np.ones((len(points), 1))))

    ddx, ddy, dda = 0, 0, 0 # The translation and rotation to be returned
    normal_vec = find_normals(line_model) # The normal vector of each line segment

    targets = np.zeros(len(points)) # Index of the closest line segment for each point
    dist_from_target = np.zeros(len(points))
    for i in range(len(points)):
        # Step 1 Find targets for data points (find the closest line segment to each point)
        for j, point in enumerate(points):
            dist = np.zeros(len(line_model))
            abs_dis = np.zeros(len(line_model))
            for k, line in enumerate(line_model):
                vi = point  # The point
                ui = normal_vec[k]  # The normal vector of the line segment
                z = line[0]  # A point on the line segment
                ri = np.dot(ui, z)  # The distance from the origin to the line
                yi = ri - np.dot(ui, vi)  # The distance from the point to the line
                abs_dis[k] = find_dist_to_lineseg(point, line)
                dist[k] = yi
            targets[j] = int(np.argmin(abs_dis))
            dist_from_target[j] = dist[int(targets[j])]

        # Plot the points and the line segments each colored by the closest line segment
        colors = ['r', 'g', 'b', 'y']
        for j, line in enumerate(line_model):
            plt.plot(line[:, 0], line[:, 1], colors[j] + '-')
        for j, point in enumerate(points):
            plt.plot(point[0], point[1], colors[int(targets[j])] + '.')
        plt.axis('equal')
        plt.title('Iteration {}'.format(i))
        plt.show()

        # Step 2 Setup the linear system of equations to solve for the translation and rotation using the least squares method
        vm = np.mean(points, axis=0)
        vi = points
        ui = np.array([normal_vec[int(targets[j])] for j in range(len(points))])
        xi1 = ui[:,0]
        xi2 = ui[:,1]

        # Subtract every data point with the robot position (vi - vm)
        diff = vi - vm
        m = np.array([[0, -1], [1, 0]])
        diff = np.dot(m, diff.T).T
        xi3 = np.sum(ui * diff, axis=1)

        A = np.array([xi1, xi2, xi3]).T
        y = dist_from_target
        b = np.dot(np.linalg.pinv(A), y)

        # Step 3 Update the translation and rotation
        dx = b[0]
        dy = b[1]
        da = b[2]
        ddx += dx
        ddy += dy
        dda += da

        # Calculate the covariance matrix
        n = len(points) # Number of data points
        res = y - np.dot(A, b)  # Residuals
        sigma = np.dot(res.T, res) / (n - 4)    # Variance estimate
        C = sigma * np.linalg.pinv(np.dot(A.T, A))  # Covariance matrix
        print('Covariance matrix: \n{}'.format(C))

        # Apply the translation and rotation
        R = np.array([[np.cos(dda), -np.sin(dda), ddx], [np.sin(dda), np.cos(dda), ddy], [0, 0, 1]])
        points = np.dot(point_copy, R.T)
        points = points[:, :2]

        # Step 4 Check if the translation and rotation are small enough to stop
        if np.linalg.norm([dx, dy, da]) < 1e-3:
            break


    return ddx, ddy, dda


dx, dy, da = cox_linefit(points, line_mod, 100)
print('dx: {}, dy: {}, da: {}'.format(dx, dy, da))

# Apply the translation and rotation
# add ones to the points to make the matrix multiplication easier
points = np.hstack((points, np.ones((len(points), 1))))
R = np.array([[np.cos(da), -np.sin(da), dx], [np.sin(da), np.cos(da), dy], [0, 0, 1]])
points = np.dot(points, R.T)
points = points[:, :2]

plt.plot(points[:, 0], points[:, 1], 'r.')
plt.plot(corner_points[:, 0], corner_points[:, 1], 'bo')
plt.plot(line_mod[:, :, 0], line_mod[:, :, 1], 'b-')
plt.axis('equal')
plt.title("dx: {}, dy: {}, da: {}".format(dx, dy, da))
plt.show()