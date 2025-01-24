# Moving Platform UAV Landing
 In this study, a low-cost, reliable, and high-accuracy solution is aimed to be developed using ArUco visual marker systems. The work examines the advantages and applicability of visual marker technology for autonomous UAV landings on moving platforms. Within the scope of the project, a structure based on the Ubuntu operating system and ROS (Robot Operating System) was established in the Gazebo simulation environment, and the UAV was controlled using the ArduPilot SITL software. ArUco markers were placed on the unmanned ground vehicle chosen as the moving platform, enabling the UAV to detect these markers and perform landings according to dynamic environmental conditions. Algorithms developed using the Python programming language along with the OpenCV, dronekit, and rospy libraries supported real-time detection of visual markers and the control of the landing process. The simulation results demonstrate that the system enables safe and precise landings on moving platforms while offering a low-cost and practical solution. The developed system provides an infrastructure that can increase operational efficiency, particularly in military operations and civil applications.

# Project Directory Structure

├── launch
│   ├── drone.launch           # Launch file for starting the drone simulation
├── models
│   ├── drone_with_camera      # Drone model with attached camera
│   │   ├── meshes             # Mesh files for the drone and its components
│   │   │   ├── iris.dae
│   │   │   ├── iris_prop_ccw.dae
│   │   │   └── iris_prop_cw.dae
│   │   ├── model.config       # Configuration file for the drone model
│   │   └── model.sdf          # SDF file defining the drone model
│   ├── husky                  # Unmanned ground vehicle (UGV) model
│   │   ├── meshes             # Mesh and texture files for the platform
│   │   │   ├── asphalt.jpg    # Texture for the platform surface
│   │   │   ├── marker.dae     # Marker 3D model
│   │   │   ├── marker.png     # Marker texture
│   │   │   ├── platform.dae   # 3D model of the platform
│   │   └── urdf
│   │       └── ugv.urdf.xacro # URDF description of the UGV
├── package.xml                # Package configuration file
├── scripts
│   ├── move_platform.py       # Python script for controlling platform movement
│   └── drone_pose_controller.py # Python script for controlling drone's pose
└── worlds
    └── runway.world           # World file defining the simulation environment
