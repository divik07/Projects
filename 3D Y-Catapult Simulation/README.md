This project focuses on the design and analysis of a 3D Y-shaped catapult system, leveraging elastic beam mechanics and energy conservation principles. The system simulates the physics of projectile motion with the goal of optimizing range and trajectory. Key aspects include modeling bending and stretching forces in interconnected beams and calculating optimal release angles for maximum projectile efficiency.

The system is modeled using a set of degrees of freedom (DOF) that define the positions and displacements of each node in the structure. These nodes represent the points along the vertical beam, inclined beams, and the elastic string. By capturing the positions in three dimensions (X, Y, Z), the framework ensures a comprehensive representation of the system's geometry and deformation.

The framework calculates forces within the system based on the potential energy stored in the elastic components. This includes:

• Stretching Energy: Arising from the elongation of the beams and string. 

• Bending Energy: Due to the angular deformation of the beams.

By computing the gradients of these energy components, the framework determines the forces acting on each node. This energy-based approach ensures that the system adheres to principles of energy conservation.

## Video Representation

![FinalReport_SimulationVideos_plotting_videoPull20cm-ezgif com-video-to-gif-converter](https://github.com/user-attachments/assets/42c6e5b1-3b64-4b89-8faa-01d9d7736282)


## Key Equations

#### Equation of Motion
$$ f = m_i * q_i'' + ∂E_{potential}/∂q_i + c_i * q_i' + W = 0 $$

Where:
- $m_i$: Mass of the node.
- $q_i$: Displacement vector (X, Y, Z).
- $E_{potential}$: Total potential energy (bending + stretching).
- $c_i$: Damping coefficient.
- $W$: External forces (e.g., gravity).

#### Discretized Equation of Motion
$$ m_i * [(q_i(t_{k+1}) - q_i(t_k)) / Δt^2] + ∂E_{potential}/∂q_i + c_i * [(q_i(t_{k+1}) - q_i(t_k)) / Δt] + W = 0 $$

#### Total Potential Energy
$$ E_{potential} = Σ (E_s^a) + Σ (E_b^b) $$

Where:
- $E_s^a$: Stretching energy for element a.
- $E_b^b$: Bending energy for element b.
