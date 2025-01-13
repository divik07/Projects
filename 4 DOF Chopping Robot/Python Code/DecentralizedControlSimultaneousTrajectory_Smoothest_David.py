# Import necessary Python libraries
# Opertational 
import math
import signal
import time
from collections import deque
from collections.abc import Sequence
import numpy as np
from numpy.typing import NDArray
import matplotlib.pyplot as plt
from scipy.interpolate import CubicSpline
# Dynamixel
from dynamixel_utils.dc_motor_model import DCMotorModel
from dynamixel_utils.motor import(
    get_mx28_control_table_json_path,
    group_sync_write,
    group_sync_read,
)
from dynio.dynamixel_controller import DynamixelIO, DynamixelMotor
from dynamixel_utils import FixedFrequencyLoopManager


# Defining function to get trajectory
def getTraj(t_start, t_end, n, x_start, x_end):
    # Defining actual and trajectory timesteps
    times = np.linspace(t_start, t_end, n)
    intervals = np.linspace(t_start, t_end, n)
    # Calculating directional trajectories
    points = [[], [], [], []]
    for i in range(4):
        a_0 = x_start[i]
        a_2 = 3*(x_end[i] - x_start[i])/(t_end**2)
        a_3 = 2*(x_start[i] - x_end[i])/(t_end**3)
        points[i].append(a_3*np.power(intervals,3) + a_2*np.power(intervals,2) + a_0)
    # Reformmating directional trajectories
    points = np.stack((points[0][0], points[1][0], points[2][0], points[3][0]), axis=0)
    # Computing trajectory derivatives
    coordinates = CubicSpline(x=intervals, y=points, axis=1, bc_type="clamped")
    # Returning position, velocity, and acceleration
    return intervals, coordinates(times), coordinates(times, 1), coordinates(times, 2) 


# Defining function to get cutting trajectory
def getTraj_cutting(t_start, t_end, tb, n, x_start, x_end):
    # Defining actual and trajectory timesteps
    times = np.linspace(t_start, t_end, n)
    intervals = np.linspace(t_start, t_end, n)
    interval_xyp = np.linspace(t_start, t_end, n)
    interval_z1 = np.linspace(t_start, tb, 2)
    interval_z2 = np.linspace(tb+0.01, t_end, n)
    interval_z = np.hstack((interval_z1, interval_z2))
    # Calculating directional trajectories
    x = np.linspace(x_start[0], x_end[0], n)
    y = np.linspace(x_start[1], x_end[1], n)
    psi = np.linspace(x_start[2], x_end[2], n)
    # Caclulating parabolic variation of z with time
    z = (x_start[3] - x_end[3])/t_end**2 * interval_z ** 2 - 2*(x_start[3] - x_end[3])/t_end * interval_z + x_start[3]
    #z = np.rad2deg(np.arcsin(z/125))
    # Storing points
    points_xyp = np.stack((x, y, psi), axis=0)
    # Computing trajectory
    coordinates_xyp = CubicSpline(x=interval_xyp, y=points_xyp, axis=1, bc_type="clamped")
    coordinates_z = CubicSpline(x=interval_z, y=z, axis=1, bc_type="clamped")
    # Reformatting
    pos = np.vstack((coordinates_xyp(times), coordinates_z(times)))
    vel = np.vstack((coordinates_xyp(times,1), coordinates_z(times,1)))
    acc = np.vstack((coordinates_xyp(times, 2), coordinates_z(times, 2)))
    # Returning position, velocity, and acceleration
    return intervals, pos, vel, acc


# Defining Trajectory Class
class TrajectoryGeneration:
    # Defining initialization function
    def __init__(self, times: NDArray[np.double], coordinates: NDArray[np.double]):
        super().__init__()
        # Initializing parameters to class
        self.times = np.asarray(times, dtype=np.double)
        self.coordinates = np.asarray(coordinates, dtype=np.double)
        self.ix: int = 0

    # Defining function to be executed when called
    def __call__(self, t: float) -> NDArray[np.double]:
        # Updates which timestep is accessed
        if t >=self.times[self.ix]:
            self.ix += 1
        # Accesses the final timestep if the end is reached
        self.ix = min(self.ix, len(self.times) - 2)
        # Returns the coordinates at the current timestep
        return self.coordinates[:, self.ix]
        

# Defining Controller Class
class PDController:
    # Defining initialization function
    def __init__(
            self,
            serial_port_name: str,
            dxl_ids: tuple[int, int, int, int],
            K_P: NDArray[np.double],
            K_I: NDArray[np.double],
            K_D: NDArray[np.double],
            link_lengths = [137, 110],
    ):
        # Initializing parameters to controller
        self.K_P = np.asarray(K_P, dtype=np.double)
        self.K_I = np.asarray(K_I, dtype=np.double)
        self.K_D = np.asarray(K_D, dtype=np.double)
        self.link_lengths = link_lengths
        self.freq = 50.0
        self.period = 1 / self.freq
        self.integral_limit = 30

        # Ensuring proper communication    
        self.loop_manager = FixedFrequencyLoopManager(period_ns=round(self.period * 1e9))

        # Creating storage variables
        self.timestamps = deque()
        self.e_i = [0, 0, 0, 0]
        self.position_history = deque()
        self.location_history = deque()
        self.dx_history = deque()

        # Creating motors
        self.dxl_io = DynamixelIO(device_name=str(serial_port_name), baud_rate=57_600)
        self.dxl_ids = dxl_ids
        self.motors = np.empty(len(self.dxl_ids), dtype=DynamixelMotor)
        for i in range(len(self.dxl_ids)):
            self.motors[i] = DynamixelMotor(
                dxl_ids[i],
                self.dxl_io,
                json_file=get_mx28_control_table_json_path(),
                protocol=2,
            )
            
        # Assuming each motor is identical
        pwm_limit_add, pwm_limit_len = self.motors[0].CONTROL_TABLE["PWM_Limit"]
        pwm_limits = group_sync_read(self.dxl_io, self.dxl_ids, pwm_limit_add, pwm_limit_len,)
        velocity_limit_add, velocity_limit_len = self.motors[0].CONTROL_TABLE["PWM_Limit"]  #DOUBLE CHECK#####
        velocity_limits = group_sync_read(self.dxl_io, self.dxl_ids, velocity_limit_add, velocity_limit_len,)
        self.pwm_limits = np.empty(len(self.dxl_ids))
        self.velocity_limits = np.empty(len(self.dxl_ids))
        for i in range(len(dxl_ids)):
            self.pwm_limits[i] = pwm_limits[dxl_ids[i]]
            self.velocity_limits[i] = velocity_limits[dxl_ids[i]]
        self.motor_model = DCMotorModel(self.period, pwm_limits=self.pwm_limits)
        
        # Cleaning up
        signal.signal(signal.SIGINT, self.signal_handler)
        signal.signal(signal.SIGTERM, self.signal_handler)


    # Defining function to track pose
    def track(self, times, traj, dtraj, ddtraj, prev_start):
        # Resetting integral sum
        self.e_i = [0, 0, 0, 0]
        
        # Recording start time
        start_time = time.time()
        cur_time = start_time
        
        # Creating trajectory objects
        self.traj = TrajectoryGeneration(times, traj)
        self.dtraj = TrajectoryGeneration(times, dtraj)
        self.ddtraj = TrajectoryGeneration(times, ddtraj)

        # Continuing through trajectory
        while cur_time - start_time < times[-1]:
            # Step 0: Extract Waypoint
            cur_time = time.time()
            # Getting desired position and velocity
            x = self.traj(cur_time - start_time)
            dx = self.dtraj(cur_time - start_time)
            ddx = self.ddtraj(cur_time - start_time)
            
            # Step 1: Get Feedback
            # Getting joint position and velocity
            q_cur = self.getQ()
            dq_cur = self.getdQ()
            # Converting to operational space
            x_cur = self.FK(q_cur)
            dx_cur = self.J_A(q_cur) @ dq_cur.T

            # Step 2: Calculating Error
            e_p = np.subtract(x, x_cur)
            e_d = np.subtract(dx, dx_cur)
            self.e_i = np.add(self.e_i, e_p * self.period)
            # Capping integral error
            for i in range(len(x)):
                if self.e_i[i] < -self.integral_limit:
                    self.e_i[i] = -self.integral_limit
                if self.e_i[i] > self.integral_limit:
                    self.e_i[i] = self.integral_limit

            # Step 3: Calculating Control Action
            # Scaling by gains
            y = self.K_P * e_p + self.K_D * e_d + self.K_I * self.e_i
            '''# Feedforward Action
            scale = 0.0000005
            y += scale * ddx'''
            # Converting from operational space to control effort
            u = self.J_A(q_cur).T @ y.T
            # Converting to PWM
            pwm_command = self.motor_model.calc_pwm_command(u)

            # Step 4: Sending Command
            self.sendPWM(pwm_command)

            # Step 5: Updating and Storing
            self.timestamps.append(cur_time - start_time + prev_start)
            self.position_history.append(q_cur)
            self.location_history.append(x_cur)
            self.dx_history.append(dx_cur)
            self.loop_manager.sleep()
            

    # Defining function to turn motors on
    def torqueOn(self):
        for i in range(len(self.motors)):
            self.motors[i].torque_enable()
            print("Turned on Motor ", i+1)

    # Defining function to turn motors off
    def torqueOff(self):
        for i in range(len(self.motors)):
            self.motors[i].torque_disable()
            print("Turned off Motor ", i + 1)

    # Defining function to get current joint space pose
    def getQ(self) -> NDArray[np.double]:
        (cur_pos_control_add, cur_pos_len,) = self.motors[0].CONTROL_TABLE["Present_Position"]
        joint_ticks = group_sync_read(self.dxl_io, self.dxl_ids, cur_pos_control_add, cur_pos_len,)
        q = np.empty((len(self.motors),), dtype=np.double)
        for i, motor in enumerate(self.motors):
            q[i] = self.convert_encoder_tick_to_deg(motor, joint_ticks[motor.dxl_id])
        return q

    # Defining function to get current joint space velocity
    def getdQ(self):
        (cur_vel_control_add, cur_vel_len,) = self.motors[0].CONTROL_TABLE["Present_Velocity"]
        joint_vel_ticks = group_sync_read(self.dxl_io, self.dxl_ids, cur_vel_control_add, cur_vel_len,)
        dq = np.empty((len(self.motors),), dtype=np.double)
        for i, motor in enumerate(self.motors):
            dq[i] = self.convert_velocity_tick_to_deg_per_s(joint_vel_ticks[motor.dxl_id])
        return dq
    
    # Defining function to send PWM signals
    def sendPWM(self, pwm_commands: NDArray[np.int64]):
        goal_pwm_add, goal_pwm_len = self.motors[0].CONTROL_TABLE["Goal_PWM"]
        pwm = {}
        for i in range(len(self.motors)):
            pwm[self.dxl_ids[i]] = pwm_commands[i]
        group_sync_write(self.dxl_io, pwm, goal_pwm_add, goal_pwm_len,)

    # Defining function to calculate forward kinematics
    def FK(self, q_cur):
        # Accounting for offsets and converting to radians
        q_cur = np.deg2rad(q_cur)
        offset = np.deg2rad([89, 188, 186, 180])
        q_cur = q_cur - offset
        # Accounting for end effector shape
        a = 70
        b = 125 * np.cos(q_cur[3])
        L3 = np.sqrt(a**2 + b**2)
        phi = np.arctan(b/a)
        # Calculating position and orientation
        x = self.link_lengths[0] * np.cos(q_cur[0]) + self.link_lengths[1] * np.cos(q_cur[0] + q_cur[1]) + L3 * np.cos(q_cur[0] + q_cur[1] + q_cur[2] + phi)
        y = self.link_lengths[0] * np.sin(q_cur[0]) + self.link_lengths[1] * np.sin(q_cur[0] + q_cur[1]) + L3 * np.sin(q_cur[0] + q_cur[1] + q_cur[2] + phi)
        psi = np.rad2deg(q_cur[0] + q_cur[1] + q_cur[2] + np.pi/2)
        # Returning vector of current pose in operational space
        x_cur = [x, y, psi, np.rad2deg(q_cur[3])]
        return x_cur

    # Defining function to calculate Analytical Jacobian
    def J_A(self, q_cur):
        # Accounting for offsets and converting to radians
        q_cur = np.deg2rad(q_cur)
        offset = np.deg2rad([89, 188, 186, 180])
        q_cur = q_cur - offset
        # Accounting for end effector shape
        a = 70
        b = 125 * np.cos(q_cur[3])
        L3 = np.sqrt(a**2 + b**2)
        phi = np.arctan(b/a)
        # Filling each index of Jacobian matrix
        J_A = np.zeros([4, 4])
        J_A[0, 0] = -self.link_lengths[0]*np.sin(q_cur[0]) - self.link_lengths[1]*np.sin(q_cur[0] + q_cur[1]) - L3 * np.sin(q_cur[0] + q_cur[1] + q_cur[2] + phi)
        J_A[0, 1] = -self.link_lengths[1] * np.sin(q_cur[0] + q_cur[1]) - L3 * np.sin(q_cur[0] + q_cur[1] + q_cur[2] + phi)
        J_A[0, 2] = - L3 * np.sin(q_cur[0] + q_cur[1] + q_cur[2] + phi)
        J_A[1, 0] = self.link_lengths[0] * np.cos(q_cur[0]) + self.link_lengths[1] * np.cos(q_cur[0] + q_cur[1]) + L3 * np.cos(q_cur[0] + q_cur[1] + q_cur[2] + phi)
        J_A[1, 1] = self.link_lengths[1] * np.cos(q_cur[0] + q_cur[1]) + L3 * np.cos(q_cur[0] + q_cur[1] + q_cur[2] + phi)
        J_A[1, 2] = L3 * np.cos(q_cur[0] + q_cur[1] + q_cur[2] + phi)
        J_A[2, 0] = J_A[2, 1] = J_A[2, 2] = J_A[3, 3] = 1
        return J_A

    # Defining function to convert encoder readings to degrees    
    @staticmethod
    def convert_encoder_tick_to_deg(motor: DynamixelMotor, pos_tick: int):
        pos_tick = np.asarray(pos_tick).astype(np.int16)
        if pos_tick > 65536:
            pos_tick = -np.invert(pos_tick)
        ang = motor.max_angle * (pos_tick - motor.min_position) / (motor.max_position + 1 - motor.min_position)
        return ang

    # Defining function to convert encoder readings to velocity
    @staticmethod
    def convert_velocity_tick_to_deg_per_s(velocity: int):
        vel_tick = np.asarray(velocity).astype(np.int16)
        if vel_tick > 65536:
            vel_tick = -np.invert(vel_tick)
        rpm = 6 * (float(vel_tick) * 0.229)
        return rpm

    # Defining function to throw error if Dynamixel connection fails
    def signal_handler(self, *_):
        self.torqueOff()


# Defining main function
if __name__ == "__main__":
    ########## Creating Controller ##########
    # Defining name of COM port to U2D2
    serial_port_name = "COM4"

    # Defining motor IDs
    dxl_ids = 1, 2, 3, 4

    # Defining gain matrices
    K_P = np.array([260e-7, 160e-7, 150e-5, 20e-3]) #np.array([5e-5, 19e-6, 29e-4, 3e-2])
    K_I = np.array([90e-8, 100e-8, 180e-5, 100e-4]) #np.array([9e-6, 90e-7, 40e-4, 55e-3])
    K_D = np.array([40e-10, 190e-10, 180e-7, 35e-5]) #np.array([4e-8, 21e-9, 29e-7, 16e-4])

    # Defining Controller
    c = PDController(
        serial_port_name=serial_port_name,
        K_P=K_P,
        K_I=K_I,
        K_D=K_D,
        dxl_ids=dxl_ids,
    )


    ########## Trajectory Generation ##########
    # Defining intermediate positions
    home = [-50, 185, 215, 40]
    home_via = [-150, 25, 270, 40]
    cut1_via = [-175, 15, 270, 20]
    cut1 = [-175, 60, 270, -10]
    cut1_post = [-150, 70, 270, 30]
    cut2_via = [-150, 15, 270, 20]
    cut2 = [-150, 60, 270, -10]
    cut2_post = [-150, 70, 270, 30]
    cut3_via = [-125, 15, 270, 20]
    cut3 = [-125, 60, 270, -10]
    cut3_post = [-125, 70, 270, 30]
    move_via = [-100, 80, 270, 30]
    move_start = [-100, 53, 270, -5]
    move_end = [-200, 53, 270, -5]

    n = 19
    # Creating trajectory
    t1, x1, dx1, ddx1 = getTraj(0, 3, 3*n, home, home_via)
    t2, x2, dx2, ddx2 = getTraj(0, 1, 1*n, home_via, cut1_via)
    t3, x3, dx3, ddx3 = getTraj_cutting(0, 3, 0.5, 3*n, cut1_via, cut1)
    t4, x4, dx4, ddx4 = getTraj(0, 1, 1*n, cut1, cut1_post)
    t5, x5, dx5, ddx5 = getTraj(0, 1, 1*n, cut1_post, cut2_via)
    t6, x6, dx6, ddx6 = getTraj_cutting(0, 3, 0.5, 3*n, cut2_via, cut2)
    t7, x7, dx7, ddx7 = getTraj(0, 1, 1*n, cut2, cut2_post)
    t8, x8, dx8, ddx8 = getTraj(0, 1, 1*n, cut2_post, cut3_via)
    t9, x9, dx9, ddx9 = getTraj_cutting(0, 3, 0.5, 3*n, cut3_via, cut3)
    t10, x10, dx10, ddx10 = getTraj(0, 1, 1*n, cut3, cut3_post)
    t11, x11, dx11, ddx11 = getTraj(0, 1, 1*n, cut3_post, move_via)
    t12, x12, dx12, ddx12 = getTraj(0, 2, 2*n, move_via, move_start)
    t13, x13, dx13, ddx13 = getTraj(0, 3, 3*n, move_start, move_end)
    t14, x14, dx14, ddx14 = getTraj(0, 2, 2*n, move_end, home_via)
    t15, x15, dx15, ddx15 = getTraj(0, 3, 3*n, home_via, home)

    # Storing trajectories together
    times = [t1, t2, t3, t4, t5, t6, t7, t8, t9, t10, t11, t12, t13, t14, t15]
    trajectories = [x1, x2, x3, x4, x5, x6, x7, x8, x9, x10, x11, x12, x13, x14, x15]
    d_trajectories = [dx1, dx2, dx3, dx4, dx5, dx6, dx7, dx8, dx9, dx10, dx11, dx12, dx13, dx14, dx15]
    dd_trajectories = [ddx1, ddx2, ddx3, ddx4, ddx5, ddx6, ddx7, ddx8, ddx9, ddx10, ddx11, ddx12, ddx13, ddx14, ddx15]


    ########## Tracking Trajectory ##########
    # Starting motors, running, and stopping
    c.torqueOn()
    t_prev = 0
    for i in range(0, len(times)):
        if i > 0:
            t_prev += times[i-1][-1]
        c.track(times[i], trajectories[i], d_trajectories[i], dd_trajectories[i], t_prev)
    c.torqueOff()


    ########## Printing Results ##########
    # Saving output
    tim = np.asarray(c.timestamps)
    pos = np.asarray(c.position_history).T
    loc = np.asarray(c.location_history).T

    # Plotting joint space trajectories
    fig1, axs1 = plt.subplots(len(c.motors))
    for i in range(len(c.motors)):
        axs1[i].plot(tim, pos[i])
        axs1[i].set_ylabel('Motor ' +  str(i+1) + ' Angle (deg)', fontsize=8)
    plt.xlabel("Time (s)")

    # Plotting operational space trajectories
    labels = ['X', 'Y', 'Phi', 'Psi']
    units = ['mm', 'mm', 'deg', 'deg']
    fig2, axs2 = plt.subplots(len(c.motors))
    for i in range(len(c.motors)):
        axs2[i].plot(tim, loc[i])
        time_add = 0
        for j in range(0, len(times)):
            if j > 0:
                time_add += times[j-1][-1]
            axs2[i].plot(times[j][0::1] + time_add, trajectories[j][i], linestyle='--', color='green')
        axs2[i].set_ylabel(labels[i] + ' Position (' + units[i] + ')', fontsize=8)
    plt.xlabel("Time (s)")
    plt.suptitle("Operational Space Trajectories")

    # Plotting overhead view
    plt.figure(3)
    plt.plot(loc[0][0], loc[1][0], marker='o', color='red')
    plt.plot(loc[0][-1], loc[1][-1], marker='x', color='red')
    plt.plot(loc[0], loc[1])
    for j in range(0, len(times)):
        plt.plot(trajectories[j][0], trajectories[j][1], linestyle='--', color='green')
    plt.title("Overhead View")
    plt.xlabel("X Position (mm)")
    plt.ylabel("Y Position (mm)")
    plt.axis('equal')
    plt.show()