import numpy
import numpy as np
from matplotlib import pyplot as plt


#Calculate IK based on the input px,py and pz values
def CalculateIK(px, py, pz):
    a1 = (pz - 34.25) / 86
    t4 = np.arctan2(a1, np.sqrt(1 - a1 ** 2))  #calculates theta 4

    k1 = py - 86 * np.cos(t4) - 31.1
    k2 = px - 164.19

    b1 = (k1 ** 2 + k2 ** 2 - 110.5 ** 2 - 113 ** 2) / (2 * 110.5 * 113)

    t2 = np.arctan2(np.sqrt(1 - b1 ** 2), b1)  #calculates theta 2

    b = 110.5 * np.cos(t2) + 113
    a = 110.5 * np.sin(t2)
    c = k1

    t1 = np.arctan2(b, a) - np.arctan2(np.sqrt(a ** 2 + b ** 2 - c ** 2), c)  #calculates theta 1

    t3 = np.pi / 2 - t1 - t2  #calculates theta 4

    return t1, t2, t3  #return joint angles in radian


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

    y = np.linspace(-30, 30, 100)
    #translates from -30 to 30 mm to achieve slicing
    z = 0.0133 * y**2 - 0.8 *y + 34 # Defines a parabolic function

    # For slicing assume constant x=278.54
    [t1, t2, t3, t4] = CalculateIK(278.54, y, z) # compute joint position
    time = np.linspace(1, 4, 100)
    # For joint 1
    t1d = np.gradient(t1, time) # compute joint velocity
    t1dd = np.gradient(t1d, time) # compute joint acceleration

    #For Joint 2
    t2d = np.gradient(t2, time)  # compute joint velocity
    t2dd = np.gradient(t2d, time)  # compute joint acceleration

    #For Joint 3
    t3d = np.gradient(t3, time)  # compute joint velocity
    t3dd = np.gradient(t3d, time)  # compute joint acceleration

    #For Joint 4
    t4d = np.gradient(t4, time)  # compute joint velocity
    t4dd = np.gradient(t4d, time)  # compute joint acceleration

    q = np.column_stack((t1, t2, t3, t4))
    qd = np.column_stack((t1d, t2d, t3d, t4d))
    qdd = np.column_stack((t1dd, t2dd, t3dd, t4dd))

    return q,qd,qdd

def traj_Cutting_Continued():
    # Defines a straight line to continue the cut continued from the last point.
    # Arm must stop after parabolic cutting and then follow this traj to avoid jerks

    y_end = 30 # end y position of cutting traj
    z_end = 0.0133 * y_end**2 - 0.8 *y_end + 34 # end z

    # Define a straight line cut
    y = np.linspace(30, 70, 20)
    z = z_end*np.ones(20)
    [t1, t2, t3, t4] = CalculateIK(278.54, y, z)  # compute joint position
    time = np.linspace(1, 3, 20)
    # For joint 1
    t1d = np.gradient(t1, time)  # compute joint velocity
    t1dd = np.gradient(t1d, time)  # compute joint acceleration

    # For Joint 2
    t2d = np.gradient(t2, time)  # compute joint velocity
    t2dd = np.gradient(t2d, time)  # compute joint acceleration

    # For Joint 3
    t3d = np.gradient(t3, time)  # compute joint velocity
    t3dd = np.gradient(t3d, time)  # compute joint acceleration

    # For Joint 4
    t4d = np.gradient(t4, time)  # compute joint velocity
    t4dd = np.gradient(t4d, time)  # compute joint acceleration

    q = np.column_stack((t1, t2, t3, t4))
    qd = np.column_stack((t1d, t2d, t3d, t4d))
    qdd = np.column_stack((t1dd, t2dd, t3dd, t4dd))

    return q, qd, qdd

