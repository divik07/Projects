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
# Dynamixel
from dynamixel_utils.dc_motor_model import DCMotorModel
from dynamixel_utils.motor import(
    get_mx28_control_table_json_path,
    group_sync_write,
    group_sync_read,
)
from dynio.dynamixel_controller import DynamixelIO, DynamixelMotor
from dynamixel_utils import FixedFrequencyLoopManager

import dynio.dynamixel_controller as dxl



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
        self.freq = 30.0
        self.period = 1 / self.freq
        self.thresh = 2
        self.range = 3

        # Ensuring proper communication    
        self.loop_manager = FixedFrequencyLoopManager(period_ns=round(self.period * 1e9))

        # Creating storage variables
        self.timestamps = deque()
        self.e_i = [0, 0, 0, 0]
        self.position_history = deque()
        self.location_history = deque()
        self.tolerance = deque(maxlen=self.range)

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
    def track(self, x):
        # Recording start time
        start_time = time.time()
        add_time = False
        if len(self.timestamps) >= 1:
            added_time = self.timestamps[-1]
            add_time = True
        # Looping through motors to ensure desired positions are in range FIXXXXXX
        x_max = y_max = sum(self.link_lengths)
        phi_max = 200
        psi_max = 40
        total_max = [x_max, y_max, phi_max, psi_max]
        for i in range(len(self.motors)):
            troprp = 1
            #x[i] = max(min(x[i], total_max[i]), -total_max[i])
            # ADD condition to ensure phi >=0
        # Resetting integral sum
        self.e_i = [0, 0, 0, 0]
        self.tolerance.clear()

        # Continuing until positions are within threshold
        should_continue = True
        while should_continue:
            # Step 1: Get Feedback
            # Getting joint position and velocity
            q_cur = self.getQ()
            dq_cur = self.getdQ()
            # Converting to operational space
            x_cur = self.FK(q_cur)
            dx_cur = self.J_A(q_cur) @ dq_cur.T

            # Step 2: Calculating Error
            e_p = np.subtract(x, x_cur)
            e_d = -dx_cur
            self.e_i = np.add(self.e_i, e_p * self.period)
            
            # Step 3: Calculating Control Action
            # Scaling by gains
            u = self.K_P * -e_p - self.K_D * -e_d - self.K_I * self.e_i

            # Converting to PWM
            pwm_command = self.motor_model.calc_pwm_command(u)

            # Step 4: Sending Command
            self.sendPWM(pwm_command)

            # Step 5: Checking For Convergence            
            # Checking for convergence in each DOF
            if(len(self.tolerance) == self.range):
                turn_off = True
                for i in range(len(x_cur)):
                    if turn_off:
                        s = 0
                        for j in range(self.range):
                            s += self.tolerance[j][i]
                        if np.abs(s/self.range) > self.thresh:
                            turn_off = False
                # Exiting while loop if every motor has converged
                if turn_off:
                    should_continue = False

            # Step 6: Updating and Storing
            t_cur = time.time() - start_time
            if add_time:
                t_cur += added_time
            self.timestamps.append(t_cur)
            self.position_history.append(q_cur)
            self.location_history.append(x_cur)
            self.tolerance.append(e_p)
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
        b = 125
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
        b = 125
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
    # Defining name of COM port to U2D2
    serial_port_name = "COM4"

    # Defining motor IDs
    dxl_ids = 1, 2, 3, 4

    # Defining gain matrices
    K_P = np.array([0.002, 0.001, -0.001, -0.003])
    K_I = np.array([0.003, 0.003, -0.003, -0.003])
    K_D = np.array([0.000001, 0.000001, 0.0000008, 0.000004])

    # Defining Controller
    c = PDController(
        serial_port_name=serial_port_name,
        K_P=K_P,
        K_I=K_I,
        K_D=K_D,
        dxl_ids=dxl_ids,
    )
#DOWN 160 - UP 220
    # Defining trajectory
    x = [0, 260, 180, 0]
    y = [-50, 250, 180, 40]
    z = [-50, 250, 180, -15]
    a = [0, 250, 180, -15]

    # Starting motors, running, and stopping
    c.torqueOn()
    for i in range(0, 100, 10):
        c.track([i, 260, 180, 0])
    c.torqueOff()

    # Saving output
    t = np.asarray(c.timestamps)
    pos = np.asarray(c.position_history).T
    loc = np.asarray(c.location_history).T

    # Printing results
    if len(c.motors) == 1:
        plt.plot(t, pos)
        plt.ylabel("Motor Angle")
        plt.xlabel("Time (s)")
        plt.show()
    else:
        # Plotting positions in joint space
        fig, axs = plt.subplots(len(c.motors))
        for i in range(len(c.motors)):
            axs[i].plot(t, pos[i])
            axs[i].set_ylabel('Motor ' +  str(i+1) + ' Angle (deg)', fontsize=8)
            axs[i].axhline(x[i], linestyle='--', color='green')
        plt.xlabel("Time (s)")

        # Plotting location in operational space
        fig2, axs2 = plt.subplots(len(c.motors))
        labels = ['X', 'Y', 'Phi', 'Psi']
        units = ['mm', 'mm', 'deg', 'deg']
        for i in range(len(c.motors)):
            axs2[i].plot(t, loc[i])
            axs2[i].set_ylabel(labels[i] + ' Position (' + units[i] + ')', fontsize=8)
            axs2[i].axhline(x[i], linestyle='--', color='green')
            axs2[i].axhline(y[i], linestyle='--', color='red')
            axs2[i].axhline(z[i], linestyle='--', color='blue')
            axs2[i].axhline(a[i], linestyle='--', color='purple')
        plt.xlabel("Time (s)")

        # Plotting overhead view
        plt.figure(3)
        #plt.plot(x, y, linestyle='--')
        plt.plot(loc[0], loc[1])
        plt.axis('equal')
        plt.show()
        
