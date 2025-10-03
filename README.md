# Overview

The goal of this project is to implement a camera calibration algorithm to establish the link 
between the base of a robot arm and the camera. 

For that purpose, ArUco marker detection and pose estimation will be used to compute the TF between base_link and camera_link.  

To test the TF computation, circle patterns will be detected in the camera's video feed and the robot arm will have to go towards them. 

The whole process will be done via a simple web interface. The Javascript code for the webpage can be found on the following repo: 
https://github.com/habartakh/final_project_website.

Before starting, please install the following Python libraries:
```
cd ~/ros2_ws/src/final_project/
pip install -r requirements.txt
``` 

For simplification purposes, bash scripts were added to start up the necessary nodes.

For the webpage packages, run: 
```
cd ~/ros2_ws/src/final_project/
./launch_webpage_nodes.sh
```

For the moveit services and packages as well as the ArUco and Circle detection nodes, please run:
```
cd ~/ros2_ws/src/final_project/
./launch_ros2_nodes.sh
```

For the camera calibration packages, run:
```
cd ~/ros2_ws/src/final_project/
./launch_ros2_nodes_calibration.sh
```

Please note that the real robot arm uses Zenoh to broadcast camera frames and publish camera topics.
To install it, run:

```
cd ~/ros2_ws/src/zenoh-pointcloud/; ./install_zenoh.sh
```

A specific bash script was also made for the simulated robot packages:
```
cd ~/ros2_ws/src/final_project/
./launch_ros2_nodes_sim.sh
```

The launch files in this script use arguments specific to the simulated robot frames. 
