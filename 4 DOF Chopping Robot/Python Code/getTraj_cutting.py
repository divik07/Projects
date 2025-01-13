import numpy
import numpy as np
from matplotlib import pyplot as plt
from scipy.signal import savgol_filter
from scipy.interpolate import CubicSpline


def getTraj_cutting(t_start, t_end, tb, n, x_start, x_end):
    #tb = blend time
    #This function blends only z, different approach needs to be taken for x,y and psi like a cubic polynomial
    #CubicSpine approach might not work well for x,y and psi blending

    #Defines coordinates_z1 from 0 to tb with 2 data points to control slope effectively for z only
    #Defines coordinates_z2 from tb to t_end for z only
    interval_z1 = np.linspace(t_start, tb, 2)
    interval_z2 = np.linspace(tb+0.01, t_end, n)
    interval_z = np.hstack((interval_z1, interval_z2))
    interval_xyp = np.linspace(t_start, t_end, n)

    times = np.linspace(t_start, t_end, int(n/10))

    intervals = np.linspace(t_start, t_end, n)
    # Calculating points along each direction
    x = np.linspace(x_start[0], x_end[0], n)
    y = np.linspace(x_start[1], x_end[1], n)
    psi = np.linspace(x_start[2], x_end[2], n)
    #calculated parabolic variation of z with time
    z = (x_start[3] - x_end[3])/t_end**2 * interval_z ** 2 - 2*(x_start[3] - x_end[3])/t_end * interval_z + x_start[3]
    # Storing points
    points_xyp = np.stack((x, y, psi), axis=0)
    # Computing trajectory
    coordinates_xyp = CubicSpline(x=interval_xyp, y=points_xyp, axis=1, bc_type="clamped")
    coordinates_z = CubicSpline(x=interval_z, y=z, axis=1, bc_type="clamped")

    pos = np.vstack((coordinates_xyp(times), coordinates_z(times)))
    vel = np.vstack((coordinates_xyp(times,1), coordinates_z(times,1)))
    acc = np.vstack((coordinates_xyp(times, 2), coordinates_z(times, 2)))

    # Returning position, velocity, and acceleration

    # Plot the original data and spline interpolation
    plt.subplot(1, 2, 1)
    plt.plot(times, pos.T[:, -1], label='Original data')
    plt.xlabel('Time')
    plt.ylabel('Joint Velocity (t1d)')
    plt.title('Joint Velocity Profile')
    plt.legend()
    plt.grid(True)
    plt.show()

    return intervals, pos, vel, acc

#getTraj_cutting(0,3,0.75,1000,[40, 185, 180, 40], [80, 185, 215, -10])