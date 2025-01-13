# Import necessary Python libraries
import signal
import time
from collections import deque
import dynio.dynamixel_controller as dxl
import matplotlib.pyplot as plt
import numpy as np
from mechae263C_helpers.minilab1 import FixedFrequencyLoopManager

# Defining Controller Class
class PDController:
    # Defining initialization function
    def __init__(
            self,
            motors: [dxl.DynamixelMotor, dxl.DynamixelMotor, dxl.DynamixelMotor],
            proportional_gains: float = [0, 0, 0],
            derivative_gains: float = [0, 0, 0],
            control_freq_Hz: float = 100.0,
            link_lengths = [137, 110],
    ):
        # Initializing parameters to controller
        self.motors = motors
        self.proportional_gains = proportional_gains
        self.derivative_gains = derivative_gains
        self.link_lengths = link_lengths

        # Making sure Dynamixels are communicating
        self.loop_manager = FixedFrequencyLoopManager(control_freq_Hz)
        signal.signal(signal.SIGINT, self.signal_handler)

        # Initializing time variable and time step
        self.timestamp = 0.0
        self.dt = self.loop_manager.period_s

        # Defining vector and size used to determine average error threshold
        t_thresh = 3*self.dt
        self.num_convergence_samples = round(t_thresh / self.dt)
        self.convergence_window = [[], [], []]
        for i in range(len(motors)):
            self.convergence_window[i].append(deque(maxlen=self.num_convergence_samples))

        # Creating storage vectors for time, position, location, and errors
        self.timestamps = deque()
        self.position_history = [[], [], []] #np.empty(len(motors))
        self.location_history = [[], [], []] #np.empty(len(motors))
        self.error = [[], [], []] #np.empty(len(motors))
        self.error_derivative = [[], [], []] #np.empty(len(motors))
        for i in range(len(motors)):
            #self.position_history[i].append(deque())
            #self.location_history[i].append(deque())
            self.error[i].append(deque())
            self.error_derivative[i].append(deque())

        # Set each motor to use PWM mode and limits
        self.pwm_limit = float(self.motors[0].read_control_table("PWM_Limit"))
        for i in range(len(motors)):
            motors[i].write_control_table("Operating_Mode", 16)

        # Display initial positions
        for i in range(len(motors)):
            print(abs(self.motors[i].get_angle()))

    # Defining function to track pose
    def track(self, x):
        # Tracking segment time
        start_time = self.timestamp
        # Looping through motors to ensure desired positions are in range FIXXXXXX
        x_max = y_max = sum(self.link_lengths)
        phi_max = 180
        total_max = [x_max, y_max, phi_max]
        for i in range(len(motors)):
            x[i] = max(min(x[i], total_max[i]), -total_max[i])
        # Resetting convergence windows
        for i in range(len(motors)):
            self.convergence_window[i][0].clear()

        # Continue unitl positions are within threshold
        should_continue = True
        while should_continue:
            #### Step 1: Getting Feedback ####
            # Getting position and converting to operational space
            q_cur = self.getQ()
            x_cur = self.FK(q_cur)
            # Calculating positional error
            e_p = np.multiply(np.subtract(x_cur, x), self.proportional_gains)
            # Calculating velocity and converting to operational space
            dq_cur = self.getdQ(q_cur)
            dx_cur = self.J_A(q_cur) @ dq_cur.T
            # Calculating derivative error
            e_d = np.multiply(np.subtract(dx_cur, np.zeros(len(self.motors))), self.derivative_gains)
            # Defining total error
            e = -e_p #- e_d
            #print(np.subtract(x_cur, x))

            #### Step 2: Getting Control Effort ####
            u = self.J_A(q_cur).T @ e.T
            # Ensuring PWM signal is within bounds
            for i in range(len(motors)):
                u[i] = round(max(min(u[i], self.pwm_limit), -self.pwm_limit))

            #### Step 3: Sending Control Signal ####
            self.sendPWM(u)

            ### Step 4: Checking for Convergence ####
            # Flag to check if total convergence is reached
            should_continue = True
            turn_off = False
            ex = False
            # Checking for convergence at each motor
            for i in range(len(motors)-1):
                if not ex: 
                    #print(str(i) + " : " + str(np.abs(np.mean(np.abs(self.convergence_window[i]))) - x[i]))
                    if len(self.convergence_window[i][0]) == self.num_convergence_samples: #or self.timestamp - start_time >= 0.1:
                        if np.abs(np.mean(self.convergence_window[i]) - x[i]) < 6: #or self.timestamp - start_time >= 0.1:
                            turn_off = True
                        else:
                            turn_off = False
                            ex = True

            # Exiting while loop if every motor has converged
            if turn_off:
                should_continue = False
                
            #### Step 5: Updates and Storage ####     
            # Storing time and position
            self.timestamps.appendleft(self.timestamp)
            for i in range(len(motors)):
                self.position_history[i].insert(0, q_cur[i])
                self.location_history[i].insert(0, x_cur[i])
                self.convergence_window[i][0].append(x_cur[i])
            # Updating current time
            self.loop_manager.sleep()
            self.timestamp += self.dt

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
    def getQ(self):
        q = np.zeros(len(self.motors))
        for i in range(len(self.motors)):
            q[i] = self.motors[i].get_angle()
        return q

    # Defining function to get current joint space velocity
    def getdQ(self, q_cur):
        dq_cur = np.zeros(len(self.motors))
        if len(self.position_history[0]) > 1:
            for i in range(len(self.motors)):
                dq_cur[i] = ((self.position_history[i][-1] - q_cur[i]) / self.dt)
        return dq_cur

    # Defining function to calculate forward kinematics
    def FK(self, q_cur):
        # Accounting for offsets and converting to radians
        q_cur = np.deg2rad(q_cur)
        offset = [np.pi/2, np.pi, np.pi]
        q_cur = q_cur - offset
        # Calculating position and orientation
        x = self.link_lengths[0] * np.cos(q_cur[0]) + self.link_lengths[1] * np.cos(q_cur[0] + q_cur[1])
        y = self.link_lengths[0] * np.sin(q_cur[0]) + self.link_lengths[1] * np.sin(q_cur[0] + q_cur[1])
        psi = np.rad2deg(q_cur[0] + q_cur[1] + q_cur[2])
        # Returning vector of current pose in operational space
        x_cur = [x, y, psi]
        return x_cur

    # Defining function to calculate Analytical Jacobian
    def J_A(self, q_cur):
        # Accounting for offsets and converting to radians
        q_cur = np.deg2rad(q_cur)
        offset = [np.pi/2, np.pi, np.pi]
        q_cur = q_cur - offset
        # Filling each index of Jacobian matrix
        J_A = np.zeros([3, 3])
        J_A[0, 0] = -self.link_lengths[0]*np.sin(q_cur[0]) - self.link_lengths[1]*np.sin(q_cur[0] + q_cur[1])
        J_A[0, 1] = -self.link_lengths[1] * np.sin(q_cur[0] + q_cur[1])
        J_A[1, 0] = self.link_lengths[0] * np.cos(q_cur[0]) + self.link_lengths[1] * np.cos(q_cur[0] + q_cur[1])
        J_A[1, 1] = self.link_lengths[1] * np.cos(q_cur[0] + q_cur[1])
        J_A[2, 0] = J_A[2, 1] = J_A[2, 2] = 1
        return J_A

    # Defining function to convert encoder readings to degrees
    def convert_encoder_tick_to_deg(self, encoder_tick: int):
        return (((encoder_tick - self.motors[0].min_position) /
                 ((self.motors[0].max_position + 1) - self.motors[0].min_position)) * self.motors[0].max_angle)

    # Defining function to send PWM signal
    def sendPWM(self, u):
        for i in range(len(self.motors)):
            # Writing PWM command to each motor
            self.motors[i].write_control_table("Goal_PWM", int(u[i]))
            # Printing position of each motor
            #print("Motor ", i+1, ": ", self.convert_encoder_tick_to_deg(self.motors[i].read_control_table("Present_Position")))
        #print()

    # Defining function to throw error if Dynamixel connection fails
    def signal_handler(self, *_):
        time.sleep(self.loop_manager.period_s)
        for i in range(len(self.motors)):
            self.motors[i].torque_disable()
        exit(-1)


# Defining main function
if __name__ == "__main__":
    # Defining USB connection to U2D2
    dxl_io = dxl.DynamixelIO(
        device_name="COM4", baud_rate=57_600
    )

    # Defining motors
    m2 = dxl.DynamixelMotor(
        dxl_id=2,
        dxl_io=dxl_io,
        protocol=2,
        json_file="./mechae263C_helpers/minilab1/MX28-AR-2.json",
    )
    m3 = dxl.DynamixelMotor(
        dxl_id=3,
        dxl_io=dxl_io,
        protocol=2,
        json_file="./mechae263C_helpers/minilab1/MX28-AR-2.json",
    )
    m4 = dxl.DynamixelMotor(
        dxl_id=4,
        dxl_io=dxl_io,
        protocol=2,
        json_file="./mechae263C_helpers/minilab1/MX28-AR-2.json",
    )
    motors = [m2, m3, m4]

    # Ensuring torques are disabled so communication can occur
    #for i in range(len(motors)):
     #       motors[i].torque_disable()

    # Defining gains
    p_gains =  [0.025, 0.045, 0.06]
    d_gains =  [0.001, 0.001, 0.001]

    # Defining PID controllers
    c1 = PDController(
        motors=motors,
        proportional_gains=p_gains,
        derivative_gains=d_gains,
    )

    # Defining Trajectory
    x = []
    y = []
    for i in range(0, 360, 5):
        x.insert(-1, 30*np.cos(np.deg2rad(i)))
        y.insert(-1, 30*np.sin(np.deg2rad(i)) + 170)

    # Running Test
    c1.torqueOn()
    for i in range(len(x)):
        c1.track([x[i], y[i], 90])
    c1.torqueOff()

    # Printing results
    if len(motors) == 1:
        plt.plot(c1.timestamps, c1.position_history[0])
        plt.ylabel("Motor Angle")
        plt.xlabel("Time (s)")
        plt.show()
    else:
        fig, axs = plt.subplots(len(c1.motors))
        for i in range(len(c1.motors)):
            axs[i].plot(c1.timestamps, c1.position_history[i])
            axs[i].set_ylabel('Motor ' +  str(i+1) + ' Angle (deg)', fontsize=8)
            #axs[i].axhline(x[i], linestyle='--', color='green')
        plt.xlabel("Time (s)")

        fig2, axs2 = plt.subplots(len(c1.motors))
        labels = ['X', 'Y', 'Phi']
        units = ['mm', 'mm', 'deg']
        for i in range(len(c1.motors)):
            axs2[i].plot(c1.timestamps, c1.location_history[i])
            axs2[i].set_ylabel(labels[i] + ' Position (' + units[i] + ')', fontsize=8)
            axs2[i].axhline(x[i], linestyle='--', color='green')
        plt.xlabel("Time (s)")

        plt.figure(3)
        plt.plot(x, y, linestyle='--')
        plt.plot(c1.location_history[0], c1.location_history[1])
        plt.axis('equal')
        plt.show()
        
