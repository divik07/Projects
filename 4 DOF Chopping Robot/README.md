This project as completed to fulfil the requirements of the course MAE263C - Controls of Robotic Systems at UCLA. 

CHOPP (Culinary Helper of Produce Preparation) is a robotic manipulator designed for cutting produce in a home setting to aid personal food preparation. The project features a four-degree-of-freedom (DOF) robot equipped with a knife as its end effector, capable of executing a reciprocating motion to slice food. The manipulator’s performance was evaluated on its ability to cut a hard-boiled egg and a banana into multiple segments, then move the slices to the edge of the cutting surface. This simplified yet functional design showcases the capabilities of decentralized PID control and trajectory generation for robotic food preparation.

**Robot Design**

CHOPP consists of three links actuated by four motors:

• Motors 1 & 2: Control the position in the x and y directions.

• Motor 3: Controls the knife’s orientation (φ) about the Z3 axis.

• Motor 4: Controls the knife angle (Ψ) about the Z4 axis.

![image](https://github.com/user-attachments/assets/9b7ef452-76d0-4c9a-988a-b825c2bd5d39)


**Trajectory and Motion Control**

The manipulator’s movement was limited to the XY plane for positioning and Z3 rotation for slicing. A combination of position and velocity control was used to generate smooth trajectories, ensuring the end effector reached predefined waypoints and executed slicing motions effectively.

Key trajectory features:

• Waypoint definition: Positions for home, above the produce, start and end of slices, and produce clearing.

• Cubic and parabolic functions: Smooth transitions between waypoints and realistic slicing motions in the YZ plane.

• Controlled timing: Each slice and associated motion completed within 10 seconds.

![image](https://github.com/user-attachments/assets/8ba197fe-b978-4422-88fa-b95d642530cc)


**Controllers**

Three controller architectures were implemented:

PID control with position feedback: Used for troubleshooting and defining homing scripts.

PID control with position and velocity feedback: Achieved smoother transitions between waypoints and enabled slicing motions.

PID with feedforward acceleration: Briefly tested but excluded from the final design due to tuning complexity and instability.

![image](https://github.com/user-attachments/assets/d88fcb2a-afd3-46bc-a513-89cc7ad93ec8)


**Forward Kinematics and Jacobian Analysis
**
Operational space control required calculating forward kinematics to transform joint space outputs into global frame positions. An analytical Jacobian was derived to convert between joint and operational space, ensuring precise control of end effector movements.

![image](https://github.com/user-attachments/assets/a2506a4b-c546-4e32-8892-dfe8e1f208ff)

**Final Result**

![image](https://github.com/user-attachments/assets/8897d14e-834b-4804-9598-dddf9a9cd167)



