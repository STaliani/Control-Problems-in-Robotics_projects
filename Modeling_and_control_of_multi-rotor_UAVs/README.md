Executable trajectories:
|![immagine](https://github.com/STaliani/Control-Problems-in-Robotics_projects/assets/104269855/0d095491-038b-4d14-a78e-6e184e71a3c8)|![immagine](https://github.com/STaliani/Control-Problems-in-Robotics_projects/assets/104269855/d676d221-5053-4d8c-89de-0ab653fff6ea)|![immagine](https://github.com/STaliani/Control-Problems-in-Robotics_projects/assets/104269855/fabfe2b5-33f4-4d7f-ad1e-1051aff6c9b3)|
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
