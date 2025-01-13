# Import necessary Python libraries
import signal
import time
from collections import deque
import dynio.dynamixel_controller as dxl
import matplotlib.pyplot as plt
import numpy as np
from mechae263C_helpers.minilab1 import FixedFrequencyLoopManager


# Defining PID class
class PIDPositionController:
    """
    This class manages a PID Position Controller
    """

    def __init__(
            self,
            motors: [dxl.DynamixelMotor, dxl.DynamixelMotor],
            proportional_gain: float = [0, 0],
            integral_gain: float = [0, 0],
            derivative_gain: float = [0, 0],
            control_freq_Hz: float = 100.0,
            ang_set_points_deg: float = [0, 0],
    ):
        # Initializing parameters to PID controller
        self.ang_set_points_deg = ang_set_points_deg
        self.motors = motors
        self.proportional_gain = proportional_gain
        self.integral_gain = integral_gain
        self.derivative_gain = derivative_gain

        # Creating storage vector for desired positions
        self.position_set_point_deg = np.zeros(len(motors))
        # Looping through motors to ensure desired positions are in range
        for i in range(len(motors)):
            self.position_set_point_deg[i] = max(min(ang_set_points_deg[i], motors[i].max_angle), 0.0)

        # Setting flag to continue control
        self.should_continue = True

        # Making sure Dynamixels are communicating
        self.loop_manager = FixedFrequencyLoopManager(control_freq_Hz)
        signal.signal(signal.SIGINT, self.signal_handler)

        # Initializing time variable and time step
        self.timestamp = 0.0
        self.dt = self.loop_manager.period_s

        # Creating storage vectors for errors
        self.error = np.zeros(len(motors))
        self.error_integral = np.zeros(len(motors))
        self.error_derivative = np.zeros(len(motors))

        # Defining vector of queues to store previous error for derivative error
        self.error_window = []
        for i in range(len(motors)):
            self.error_window.append(deque(maxlen=2))

        # Defining vector of queues and their size used to compare average error to threshold
        self.num_convergence_samples = round(0.5 / self.dt)
        self.convergence_window = []
        for i in range(len(motors)):
            self.convergence_window.append(deque(maxlen=self.num_convergence_samples))

        # Defining PWM limit for motors (ASSUMES ALL MOTORS ARE IDENTICAL)
        self.pwm_limit = float(self.motors[0].read_control_table("PWM_Limit"))

        # Creating storage vectors for time and position
        self.timestamps = deque()
        self.position_history = []
        for i in range(len(motors)):
            self.position_history.append(deque())

        # Home motors sequentially
        for i in range(len(motors)):
            self.motors[i].set_position_mode()
            self.motors[i].torque_enable()
            self.motors[i].set_angle(1)
            while abs(self.motors[i].get_angle() - 180) > 1:
                self.motors[i].set_angle(180)
                time.sleep(0.5)

        # Turn of torques
        for i in range(len(motors)):
            self.motors[i].torque_disable()

        # Display initial position
        for i in range(len(motors)):
            print(abs(self.motors[i].get_angle()))

        # Set each motor to use PWM mode (i.e. voltage control)
        for i in range(len(motors)):
            motors[i].write_control_table("Operating_Mode", 16)

    # Defining function to start controller
    def start(self, motors):
        # Turn on motor torques
        for i in range(len(motors)):
            self.motors[i].torque_enable()

        # Continue until positions are within threshold
        while self.should_continue:
            # Step 1 - Get Feedback
            # Create and initialize storage vector for current positions
            ang_deg = np.zeros(len(motors))
            for i in range(len(motors)):
                ang_deg[i] = self.motors[i].get_angle()

            # Store time and positions for plotting
            self.timestamps.appendleft(self.timestamp)
            for i in range(len(motors)):
                self.position_history[i].appendleft(ang_deg[i])

            # Step 2 - Calculate Command
            # Looping through each motor
            for i in range(len(motors)):
                # Position error calculation
                self.error[i] = self.position_set_point_deg[i] - ang_deg[i]
                # Integral error calculation
                self.error_integral[i] += self.error[i] * self.dt
                # Updating previous error
                self.error_window[i].appendleft(self.error[i])
                # Updating window used to check for convergence
                self.convergence_window[i].append(self.error[i])
                # Velocity error calculation (excluding initial condition)
                if len(self.error_window[i]) == 2:
                    self.error_derivative[i] = ((self.error_window[i][-1] - self.error_window[i][0]) / 2 / self.dt)

            # Flag to check if total convergence is reached
            turn_off = False
            # Checking for convergence at each motor
            for i in range(len(motors)):
                if len(self.convergence_window[i]) == self.num_convergence_samples:
                    if np.mean(np.abs(self.convergence_window[i])) < 0.5:
                        turn_off = True
                    else:
                        turn_off = False
                        break

            # Exiting while loop if every motor has converged
            if turn_off:
                self.stop()

            # Creating storage vector for PWM signals
            pwm_command = np.zeros(len(motors))
            # Looping through each motor
            for i in range(len(motors)):
                # Applying gains
                pwm_command[i] = (
                        self.proportional_gain[i] * self.error[i]
                        + self.integral_gain[i] * self.error_integral[i]
                        + self.derivative_gain[i] * self.error_derivative[i]
                )
                # Ensuring PWM signal is within bounds
                pwm_command[i] = round(max(min(pwm_command[i], self.pwm_limit), -self.pwm_limit))

            # Step 3 - Send Command
            # Looping through each motor
            for i in range(len(motors)):
                # Writing PWM command to each motor
                self.motors[i].write_control_table("Goal_PWM", int(pwm_command[i]))
                # Printing position of each motor
                print("Motor ", i+1, ": ",
                      self.convert_encoder_tick_to_deg(
                          motors[i], self.motors[i].read_control_table("Present_Position"))
                      )
            print()

            # Updating current time
            self.loop_manager.sleep()
            self.timestamp += self.dt

        # Turning off each motor when convergence occurs
        for i in range(len(motors)):
            self.motors[i].torque_disable()
            print("Turned off Motor ", i+1)

    # Defining function to exit the control loop
    def stop(self):
        self.should_continue = False
        time.sleep(self.loop_manager.period_s)

    # Defining function to convert encoder readings to degrees (ASSUMES ALL MOTORS ARE IDENTICAL)
    def convert_encoder_tick_to_deg(self, motor, encoder_tick: int):
        return (((encoder_tick - self.motors[0].min_position) /
                 ((self.motors[0].max_position + 1) - self.motors[0].min_position)) * self.motors[0].max_angle)

    # Defining function to throw error if Dynamixel connection fails
    def signal_handler(self, motors, *_):
        self.stop()
        for i in range(len(motors)):
            self.motors[i].torque_disable()
        exit(-1)


# Defining main function
if __name__ == "__main__":
    # Defining USB connection to U2D2
    dxl_io = dxl.DynamixelIO(
        device_name="COM5", baud_rate=57_600  #"/dev/tty.usbserial-FT3FSLY0", baud_rate=57_600
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

    # Defining PID controllers
    c1 = PIDPositionController(
        motors=motors,
        proportional_gain=[8, 8, 8],
        integral_gain=[3, 3, 3],
        derivative_gain=[0.1, 0.2, 0.15],
        ang_set_points_deg=[140, 100, 215],
    )

    # Start Controller
    c1.start(motors)

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
            axs[i].set_ylabel(["Motor ", i+1, " Angle"])
        plt.xlabel("Time (s)")
        plt.show()
