#!/usr/bin/env python3

from ikpy.chain import Chain
import ikpy.utils.plot as plot_utils

import numpy as np
import time
import math
import keyboard  # Add this library for keyboard input

import serial

# Import URDF and set initial conditions
my_chain = ikpy.chain.Chain.from_urdf_file("arm_urdf.urdf", active_links_mask=[False, True, True, True, True, True, True])
target_position = [0, 0, 0.58]
target_orientation = [-1, 0, 0]

# Define IK and plotting functions
def doIK():
    global ik
    old_position = ik.copy()
    ik = my_chain.inverse_kinematics(target_position, target_orientation, orientation_mode="Z", initial_position=old_position)

def updatePlot():
    ax.clear()
    my_chain.plot(ik, ax, target=target_position)
    plt.xlim(-0.5, 0.5)
    plt.ylim(-0.5, 0.5)
    ax.set_zlim(0, 0.6)
    fig.canvas.draw()
    fig.canvas.flush_events()

def move(x, y, z):
    global target_position
    target_position = [x, y, z]
    doIK()
    updatePlot()
    sendCommand(ik[1].item(), ik[2].item(), ik[3].item(), ik[4].item(), ik[5].item(), ik[6].item(), 1)

# Function to send commands to the robot
ser = serial.Serial('COM3', 9600, timeout=1)

def sendCommand(a, b, c, d, e, f, move_time):
    command = '0{:.2f} 1{:.2f} 2{:.2f} 3{:.2f} 4{:.2f} 5{:.2f} t{:.2f}\n'.format(math.degrees(a), math.degrees(b), math.degrees(c), math.degrees(d), math.degrees(e), math.degrees(f), move_time)
    ser.write(command.encode('ASCII'))

sendCommand(ik[1].item(), ik[2].item(), ik[3].item(), ik[4].item(), ik[5].item(), ik[6].item(), 4)

# Initialize plot
import matplotlib.pyplot as plt
fig, ax = plot_utils.init_3d_figure()
fig.set_figheight(9)
fig.set_figwidth(13)
my_chain.plot(ik, ax, target=target_position)
plt.xlim(-0.5, 0.5)
plt.ylim(-0.5, 0.5)
ax.set_zlim(0, 0.6)
plt.ion()

# Keyboard control for the robot arm
x, y, z = 0, 0.25, 0.1

def keyboard_control():
    global x, y, z
    step = 0.01

    if keyboard.is_pressed('w'):  # Move forward
        x += step
    if keyboard.is_pressed('s'):  # Move backward
        x -= step
    if keyboard.is_pressed('a'):  # Move left
        y += step
    if keyboard.is_pressed('d'):  # Move right
        y -= step
    if keyboard.is_pressed('up'):  # Move up
        z += step
    if keyboard.is_pressed('down'):  # Move down
        z -= step

    move(x, y, z)

while True:
    keyboard_control()
    time.sleep(0.05)
    if keyboard.is_pressed('esc'):  # Exit loop on ESC
        break

ser.close()
