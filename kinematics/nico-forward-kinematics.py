import ikpy.chain
import numpy as np
import ikpy.utils.plot as plot_utils
import matplotlib.pyplot
from mpl_toolkits.mplot3d import Axes3D
import time
import sys


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
    elif event.key == 'Q':
        quit()

base_el = ["torso:11", "r_shoulder_z", "right_shoulder:11", "r_shoulder_y", "right_collarbone:11", "r_arm_x", "right_upper_arm:11", "r_elbow_y", "right_lower_arm:11", "r_wrist_z", "right_wrist:11", "r_wrist_x", "right_palm:11", "r_indexfingers_x", "finger_segment:23", "r_indexfinger_1st_x", "finger_segment:13", "r_indexfinger_2nd_x", "fingertip:13" ] 
my_chain = ikpy.chain.Chain.from_urdf_file("kinematics.urdf", base_elements=base_el, active_links_mask=[False, True, True, True, True, True, True, True, True, True])
min_angles = [0, -1.308996939, -0.52359877559, -2.05948851735, -1.745, -1.396, 0, -1.309, -1.309, -1.309]
max_angles = [0, 0.959931088597, 1.7453292519943295, 1.51843644924, 0, 1.396, 0.872665, 0, 0, 0]
print("chain:")
print(my_chain)
fig = matplotlib.pyplot.figure()
ax = fig.add_subplot(111, projection='3d')
fig.canvas.mpl_connect('key_press_event', on_press)

angles = []
for i in range(8):
    angles.append((min_angles[i] + max_angles[i]) / 2)
angles.append(min_angles[7])
angles.append(min_angles[7])
position = my_chain.forward_kinematics(angles)
my_chain.plot(angles, ax)
redraw()
matplotlib.pyplot.show()

