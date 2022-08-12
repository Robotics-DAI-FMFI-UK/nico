import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import time
import sys
import socket


def redraw():
    global my_chain, ax, fig, angles, position
    matplotlib.pyplot.cla()
    my_chain.plot(angles, ax)
    matplotlib.pyplot.xlim(-0.5, 0.5)
    matplotlib.pyplot.ylim(-0.5, 0.5)
    ax.set_xlabel('A,S,D - up, Z,X,C - down')
    ax.set_title('Inverse kinematics')
    fig.canvas.draw()
    position = my_chain.forward_kinematics(angles)

def printouts():
    global pos, angles, position
    print("Position:")
    print(pos)
    print("Corrensponding angles:")
    print(angles)
    print("Forward-check: orientation of end effector:")
    print(position[:3, :3])
    print("Forward-check: position of end effector:")
    print(position[:3, 3])
    print("-----------------------")

def on_press(event):
    global angles, pos
    upkeys = ['A', 'S', 'D']
    downkeys = ['Z', 'X', 'C']

    direction = 0
    for i in range(3):
        if event.key == upkeys[i]:
            index = i
            direction = 1
            break
        elif event.key == downkeys[i]:
            index = i
            direction = -1
            break
    if direction != 0:
        val = pos[index]
        val += 0.002 * direction
        pos[index] = val
        angles = my_chain.inverse_kinematics(pos)        
        redraw()
        printouts()
        
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
pos = position[:3, 3]
print("initial position:")
print(pos)

angles = my_chain.inverse_kinematics(pos)
position = my_chain.forward_kinematics(angles)
printouts()
my_chain.plot(angles, ax)
redraw()
matplotlib.pyplot.show()


