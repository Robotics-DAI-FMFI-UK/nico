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
    ax.set_xlabel('A,S,D,F,G,H,J - up, Z,X,C,V,B,N,M - down, +/- step size')
    ax.set_title('Forward kinematics')
    fig.canvas.draw()

def angle2intangle(i):
    global angles, min_motor_angles, max_motor_angles, inverted_motors
    a = angles[i]
    a /= 3.1415926536
    b = a * 180
    if b < min_motor_angles[i]:
        a = min_motor_angles[i] / 180
    if b > max_motor_angles[i]:
        a = max_motor_angles[i] / 180
    # a is now 0..1
    a *= 2048
    a += 2048
    if a == 4096:
        a = 4095
    if inverted_motors[i]:
        a = 4095 - a
    return round(a)

def intangle2angle(i, ia):
    global angles, min_motor_angles, max_motor_angles, inverted_motors
    if inverted_motors[i]:
        ia = 4095 - ia
    ia -= 2048
    ia /= 2048
    ib = ia * 180
    if ib < min_motor_angles[i]:
        ia = min_motor_angles[i] / 180
    if ib > max_motor_angles[i]:
        ia = max_motor_angles[i] / 180
    # ia is now 0..1
    ia *= 3.1415926536
    return ia

def deg2rad(alpha):
    return 3.1415926536 * alpha / 180

def rad2deg(alpha):
    return 180 * alpha / 3.1415926536

def on_press(event):
    global angles, position, min_motor_angles, max_motor_angles, stepsize
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
        val += stepsize * direction
        if val > deg2rad(max_motor_angles[index]):
            val = deg2rad(max_motor_angles[index])
        elif val < deg2rad(min_motor_angles[index]):
            val = deg2rad(min_motor_angles[index])
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
                intangles[right_hand_motor_indexes[i + 1]] = angle2intangle(i + 1)
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
    elif (event.key == '+') and (stepsize < 0.31415926536):
        stepsize *= 1.5
    elif (event.key == '-') and (stepsize > 0.031415926536):
        stepsize /= 1.5
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
stepsize = 0.31415926536 / 2

base_el = ["torso:11", "r_shoulder_z", "right_shoulder:11", "r_shoulder_y", "right_collarbone:11", "r_arm_x", "right_upper_arm:11", "r_elbow_y", "right_lower_arm:11", "r_wrist_z", "right_wrist:11", "r_wrist_x", "right_palm:11", "r_indexfingers_x", "finger_segment:23", "r_indexfinger_1st_x", "finger_segment:13", "r_indexfinger_2nd_x", "fingertip:13" ]
my_chain = ikpy.chain.Chain.from_urdf_file("../kinematics.urdf", base_elements=base_el, active_links_mask=[False, True, True, True, True, True, True, True, True, True])

# these are taken from URDF file and correspond the real angle of the actuator in space [rad]
# we do not use them as they are too restrictive
min_angles = [0, -1.308996939, -0.52359877559, -2.05948851735, -1.745, -1.396, 0, -1.309, -1.309, -1.309]
max_angles = [0, 0.959931088597, 1.7453292519943295, 1.51843644924, 0, 1.396, 0.872665, 0, 0, 0]

# these are taken from JSON robot description file and correspond the angular position extremes of the dynamixel motors [deg]
min_motor_angles = [0, -100, -180, -140, -100, -180, -180, -180, 180, -180]
max_motor_angles = [0, 125, 179, 75, 100, 180, 180, 180, 180, 180, 180]
inverted_motors = [False, False, False, True, False, True, False, True, True, True]

print("chain:")
print(my_chain)
fig = matplotlib.pyplot.figure()
ax = fig.add_subplot(111, projection='3d')
fig.canvas.mpl_connect('key_press_event', on_press)

angles = [0]
for i in range(7):
    angles.append(intangle2angle(i + 1, all_motors_initial_positions[right_hand_motor_indexes[i + 1]]))
angles.append(angles[7])
angles.append(angles[7])
position = my_chain.forward_kinematics(angles)
my_chain.plot(angles, ax)
redraw()

matplotlib.pyplot.show()

