import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import time
import sys
import socket


def redraw():
    global my_chain, ax, fig, angles
    matplotlib.pyplot.cla()
    position = my_chain.forward_kinematics(angles)
    my_chain.plot(angles, ax)
    matplotlib.pyplot.xlim(-0.5, 0.5)
    matplotlib.pyplot.ylim(-0.5, 0.5)
    ax.set_xlabel('A,S,D,F,G,H,J - up, Z,X,C,V,B,N,M - down')
    ax.set_title('Forward kinematics')
    fig.canvas.draw()

def on_press(event):
    global angles, position, min_angles, max_angles
    upkeys = [0, 'A', 'S', 'D', 'F', 'G', 'H', 'J']
    downkeys = [0, 'Z', 'X', 'C', 'V', 'B', 'N', 'M']

    direction = 0
    for i in range(8):
        if event.key == upkeys[i]:
            index = i
            direction = 1
            break
        elif event.key == downkeys[i]:
            index = i
            direction = -1
            break
    if direction != 0:
        val = angles[index]
        val += (max_angles[index] - min_angles[index]) / 10 * direction
        if val > max_angles[index]:
            val = max_angles[index]
        elif val < min_angles[index]:
            val = min_angles[index]
        if val != angles[index]:
            angles[index] = val
            if index == 7:
                angles[8] = val
                angles[9] = val
            redraw()
            position = my_chain.forward_kinematics(angles)
            print("Angles:")
            print(angles)
            print("Orientation of end effector:")
            print(position[:3, :3])
            print("Position of end effector:")
            print(position[:3, 3])
            print("-----------------------")
            intangles = []
            for i in range(22):
                intangles.append(all_motors_initial_positions[i])

            for i in range(7):
                intangles[right_hand_motor_indexes[i + 1]] = 2048 + round(2048 * angles[i + 1] / 3.1415926536)
            byteangles = [46, 0, 1, 0]
            for i in range(22):
                byteangles.append(intangles[i] % 256)
                byteangles.append(intangles[i] // 256)
            packet = bytearray(byteangles)
            try:
                sockZ.send(packet, 48)
            except:
                print("could not send packet to kinematics server")
                quit()
    elif event.key == 'Q':
        quit()

KINEMATICS_SERVER_HOST = "127.0.0.1"
KINEMATICS_SERVER_PORT = 8003
sockZ = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
try:
    print(sockZ.connect((KINEMATICS_SERVER_HOST, KINEMATICS_SERVER_PORT)))
except:
    print("Cannot connect to kinematics server, is it running?")
    quit()

right_hand_motor_indexes = [0, 16, 12, 14, 18, 10, 8, 3]
all_motors_initial_positions = [ 0, 0, 0, 0, 0, 0, 0, 0, 577, 1188, 4068, 10, 2007, 1972, 2076, 1917, 1957, 2038, 946, 2947, 2377, 1982 ];

my_chain = ikpy.chain.Chain.from_urdf_file("../kinematics.urdf", base_elements=["torso:11"], active_links_mask=[False, True, True, True, True, True, True, True, True, True])
min_angles = [0, -1.308996939, -0.52359877559, -2.05948851735, -1.745, -1.396, 0, -1.309, -1.309, -1.309]
max_angles = [0, 0.959931088597, 1.7453292519943295, 1.51843644924, 0, 1.396, 0.872665, 0, 0, 0]
print("chain:")
print(my_chain)
fig = matplotlib.pyplot.figure()
ax = fig.add_subplot(111, projection='3d')
fig.canvas.mpl_connect('key_press_event', on_press)

angles = [0]
for i in range(7):
    angles.append((all_motors_initial_positions[right_hand_motor_indexes[i + 1]] - 2048) / 2048 * 3.1415926536)
angles.append(angles[7])
angles.append(angles[7])
position = my_chain.forward_kinematics(angles)
my_chain.plot(angles, ax)
redraw()

matplotlib.pyplot.show()

