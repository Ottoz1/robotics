import paramiko
import matplotlib.pyplot as plt
import time
import os

# configure SSH connection parameters
hostname = '192.168.170.111'
username = 'pi'
password = '1234'
remote_path = '/home/pi/robotics/build/points.txt'
local_path = './points.txt'

env_measures = [3630, 2420]   # [Width X, Height Y] in mm

# create SSH client and connect to Raspberry Pi
ssh = paramiko.SSHClient()
ssh.set_missing_host_key_policy(paramiko.AutoAddPolicy())
ssh.connect(hostname=hostname, username=username, password=password)

def plot_points():
    # download the points.txt file from the Raspberry Pi
    sftp = ssh.open_sftp()
    sftp.get(remote_path, local_path)
    sftp.close()

    # open the points.txt file for reading
    with open('points.txt', 'r') as f:
        # read the contents of the file into a list of lines
        lines = f.readlines()

    # create empty lists to store x and y coordinates
    x_coords = []
    y_coords = []

    # iterate over the lines in the file
    for line in lines:
        # split the line into two values, and convert them to floats
        x, y = 0, 0
        try:
            x, y = map(float, line.split())
        except:
            return
        # append the x and y values to their respective lists
        x_coords.append(x)
        y_coords.append(y)

    # remove the points.txt after reading it
    os.remove('points.txt')

    # Check if we have less than 10 points
    if len(x_coords) < 10:
        return

    colors = ['r', 'g', 'b']

    # clear the previous plot and plot the new points
    plt.clf()
    
    # Plot all 3 points according to their color
    for i in range(3):
        plt.scatter(x_coords[i::3], y_coords[i::3], c=colors[i])


    # Plot a box around the points
    W = env_measures[0]
    H = env_measures[1]
    plt.plot([0, 0, W, W, 0], [0, H, H, 0, 0], 'k-')

    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Points')
    plt.draw()
    plt.pause(1)

# continuously update the plot and the file every second
try:
    while True:
        plot_points()

# handle the keyboard interrupt (CTRL+C) and exit gracefully
except KeyboardInterrupt:
    # check which key was pressed


    print('Program terminated by user')
    ssh.close()
    plt.close()
