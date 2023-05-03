import paramiko
import matplotlib.pyplot as plt
import time
import os

# configure SSH connection parameters
hostname = '192.168.1.24'
username = 'pi'
password = '1234'
remote_path = '/home/pi/robotics/build/points.txt'
local_path = './points.txt'

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
        x, y = map(float, line.split())
        # append the x and y values to their respective lists
        x_coords.append(x)
        y_coords.append(y)

    # remove the points.txt after reading it
    os.remove('points.txt')

    # Check if we have less than 10 points
    if len(x_coords) < 10:
        return

    # clear the previous plot and plot the new points
    plt.clf()
    plt.plot(x_coords, y_coords, 'o')
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Points')
    plt.draw()
    plt.pause(1)

# continuously update the plot and the file every second
while True:
    plot_points()

    # wait for one second before updating again
    time.sleep(1)