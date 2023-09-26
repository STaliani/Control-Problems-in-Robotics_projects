## Executable trajectories:
|![hovering](https://github.com/STaliani/Control-Problems-in-Robotics_projects/assets/85567829/8713a5ac-b500-447e-b7bd-fa949f887568)|![linear_path](https://github.com/STaliani/Control-Problems-in-Robotics_projects/assets/85567829/7d585dc5-cb6e-4234-b6b3-d31e1396e1a8)|![spyral_path](https://github.com/STaliani/Control-Problems-in-Robotics_projects/assets/85567829/1b7748ee-5d7d-427a-b3ee-3517e742396b)
|--|--|--|



## MATLAB 
This folder implements in matlab the controller of the fully actuated drone, it is also present a coppelia sim scene wich is connected to matlab scripts.

[classic_uav_simulation.m](https://github.com/STaliani/Control-Problems-in-Robotics_projects/blob/main/Modeling_and_control_of_multi-rotor_UAVs/code/matlab/classic_drone_system.m):

implements a controller for a classical quadrotor

[simulation.m](https://github.com/STaliani/Control-Problems-in-Robotics_projects/blob/main/Modeling_and_control_of_multi-rotor_UAVs/code/matlab/simulation.m):

Implements the controller for a fully actuated quadrotor provided with a tilting mechanism 

[hybrid_simulation.m](https://github.com/STaliani/Control-Problems-in-Robotics_projects/blob/main/Modeling_and_control_of_multi-rotor_UAVs/code/matlab/simulation.m):

Implements an hybrid control strategy in which the quadrotor exploits as much as possible the tilting mechanism but exploits a classical drone controller when the referenc accelerations are over the maximum achievable by tilting the propellers.

##  PYTHON
The folder contins a coppelia sim scene in which is encoded the controller for the tilting mechanism in python.
