import numpy
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import savgol_filter


#Calculate IK based on the input px,py and pz values
def CalculateIK(px, py, pz):
    a1 = (pz - 34.25) / 86
    t4 = np.arctan2(a1, np.sqrt(1 - a1 ** 2))  #calculates theta 4

    k1 = py - 86 * np.cos(t4) - 31.1
    k2 = px - 164.19

    b1 = (k1 ** 2 + k2 ** 2 - 110.5 ** 2 - 113 ** 2) / (2 * 110.5 * 113)

    t2 = np.arctan2(np.sqrt(1 - b1 ** 2), b1)  # calculates theta 2

    b = 110.5 * np.cos(t2) + 113
    a = 110.5 * np.sin(t2)
    c = k1

    t1 = np.arctan2(b, a) - np.arctan2(np.sqrt(a ** 2 + b ** 2 - c ** 2), c)  # calculates theta 1

    t3 = np.pi / 2 - t1 - t2  # calculates theta 4

    return t1, t2, t3, t4  # return joint angles in radian


def direct_kinematics(t1, t2, t3, t4):
    # Convert angles from degrees to radians
    t1 = np.deg2rad(t1)
    t2 = np.deg2rad(t2)
    t3 = np.deg2rad(t3)
    t4 = np.deg2rad(t4)

    # Compute the elements of the transformation matrix
    T = np.array([
        [np.cos(t1 + t2 + t3) * np.cos(t4), -np.cos(t1 + t2 + t3) * np.sin(t4), np.sin(t1 + t2 + t3),
         43 * np.cos(t1 + t2 + t3 - t4) + 80.6754 * np.cos(t1 + t2 + t3 - 1.1751) + 43 * np.cos(
             t1 + t2 + t3 + t4) + 110.5 * np.cos(t1 + t2) + 113 * np.cos(t1) + 93.36],
        [np.sin(t1 + t2 + t3) * np.cos(t4), -np.sin(t1 + t2 + t3) * np.sin(t4), -np.cos(t1 + t2 + t3),
         43 * np.sin(t1 + t2 + t3 - t4) - 80.6754 * np.cos(t1 + t2 + t3 + 0.3957) + 43 * np.sin(
             t1 + t2 + t3 + t4) + 110.5 * np.sin(t1 + t2) + 113 * np.sin(t1)],
        [np.sin(t4), np.cos(t4), 0, 86 * np.sin(t4) + 34.25],
        [0, 0, 0, 1]
    ])
    return T


def traj_Cutting():
    #...define trajectory of the joints in yz plane. Assumes the manipulator is already aligned
    #and ready to cut the egg. This function defines a parabolic cut

    tb = 0.5

    # Defining first cubic blend curve
    time1 = np.linspace(0, tb, 150)
    a1 = 95.88
    b1 = -92.89
    c1 = 0
    d1 = 69.97
    z1 = a1*time1**3 +b1*time1**2 + c1*time1 +d1

    # Defining parabolic cut
    time2 = np.linspace(tb, 4-tb, 300)
    off = 34 # offset of parabolic curve
    z2 = 2.99*time2**2 - 23.97 * time2 + 35.97 + off

    #Defining second cubic blend curve
    time3 = np.linspace(4-tb,4,150)
    a3 = 0.5819
    b3 = -3.4892
    c3 = -0.0170
    d3 = 40.5563
    z3 = a3*time3**3 + b3*time3**2 + c3*time3 +d3

    # Blending for x trajectory
    #Defining first blend parabola
    aa1 = 7.5/tb
    cc1 = -30 + 15*tb- aa1*tb**2
    x1 = aa1*time1**2 +cc1

    #Defining straight line
    x2 = -30 + 15*time2

    # Defining second parabolic blend
    aa3 = -7.5/tb
    bb3 = 60/tb
    cc3 = 30 - 15*tb - bb3*(4-tb) - aa3*(4-tb)**2
    x3 = aa3*time3**2 +bb3*time3 +cc3


    #Contatenate the arrays together to get complete trajectory
    z = np.concatenate((z1[:-2], z2[:-2], z3))
    x = np.concatenate((x1[:-2], x2[:-2], x3))
    time = np.concatenate((time1[:-2], time2[:-2], time3))


    # For slicing assume constant y=300
    [t1, t2, t3, t4] = CalculateIK(300,x, z)  # compute joint position

    # For joint 1
    t1d = np.gradient(t1, time)  # compute joint velocity
    t1d = savgol_filter(t1d, 51, 3)
    t1dd = np.gradient(t1d, time)  # compute joint acceleration
    t1dd = savgol_filter(t1dd, 51, 3)

    #For Joint 2
    t2d = np.gradient(t2, time)  # compute joint velocity
    t2d = savgol_filter(t2d, 61, 3)
    t2dd = np.gradient(t2d, time)  # compute joint acceleration
    t2dd = savgol_filter(t2dd, 51, 3)

    #For Joint 3
    t3d = np.gradient(t3, time)  # compute joint velocity
    t3d = savgol_filter(t3d, 51, 3)
    t3dd = np.gradient(t3d, time)  # compute joint acceleration
    t3dd = savgol_filter(t3dd, 51, 3)

    #For Joint 4
    t4d = np.gradient(t4, time)  # compute joint velocity
    t4d = savgol_filter(t4d, 51, 3)
    t4dd = np.gradient(t4d, time)  # compute joint acceleration
    t4dd = savgol_filter(t4dd, 51, 3)

    q = np.column_stack((t1, t2, t3, t4))
    qd = np.column_stack((t1d, t2d, t3d, t4d))
    qdd = np.column_stack((t1dd, t2dd, t3dd, t4dd))

    # Plot Testing
    # # Plot the joint velocity against time
    # plt.figure(figsize=(10, 5))
    #
    # plt.subplot(1, 2, 1)
    # plt.plot(time, t1d)
    # plt.xlabel('Time')
    # plt.ylabel('Joint Velocity (t1d)')
    # plt.title('Joint Velocity Profile')
    # plt.grid(True)
    #
    # # Plot the z-coordinate against time
    # plt.subplot(1, 2, 2)
    # plt.plot(time, z)
    # plt.xlabel('Time')
    # plt.ylabel('Z-coordinate')
    # plt.title('Z-coordinate Profile')
    # plt.grid(True)
    #
    # plt.tight_layout()
    # plt.show()

    return q, qd, qdd
