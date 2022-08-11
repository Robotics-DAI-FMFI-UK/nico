import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import time
import sys


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
        val += 0.02 * direction
        pos[index] = val
        angles = my_chain.inverse_kinematics(pos)
        redraw()
        printouts()
    elif event.key == 'Q':
        quit()

my_chain = ikpy.chain.Chain.from_urdf_file("kinematics.urdf", base_elements=["torso:11"], active_links_mask=[False, True, True, True, True, True, True, True, True, True])
print("chain:")
print(my_chain)
fig = matplotlib.pyplot.figure()
ax = fig.add_subplot(111, projection='3d')
fig.canvas.mpl_connect('key_press_event', on_press)

# position of end effector for all angles in the middle of range
pos = [0.35565002, -0.26645916, 0.63282545]

angles = my_chain.inverse_kinematics(pos)
position = my_chain.forward_kinematics(angles)
printouts()
my_chain.plot(angles, ax)
redraw()
matplotlib.pyplot.show()


