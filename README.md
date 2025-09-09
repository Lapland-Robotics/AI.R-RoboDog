# AI.R-RoboDog
![Arctic ai Robotics (2)](https://github.com/user-attachments/assets/f127dfc7-4975-4b46-91b3-2373ccc549fb) 

# Setup the project

This project is designed to be used with ROS Melodic under Ubuntu-18.04. It is also usable inside WSL2 using the Ubuntu-18.04 distro. 

To setup the project, first move to the folder where you want to clone the project then run the following command to clone the repo and the submodules.

```bash
git clone https://github.com/Lapland-Robotics/AI.R-RoboDog.git
cd AI.R-RoboDog
git submodule update --init --recursive
```

> Please notify any missing dependencies or error.

# Build and source the project

To build and source the project, you first need to move in the folder of the repo, then use the following command.

```bash
cd catkin_ws
catkin_make
source devel/setup.bash
```

> Please notify any build error.

# Switch Unitree Go1 to low-level mode

In order to send low-level command to the Unitree Go1, you first need to switch it to low level mode. 

First power on the robot and wait for him to stand up. Then, use the controller and press L2+A until the robot crouch, then L2+B to make him enter damping state, and finally L1+L2+Start to switch to low level mode. If you succesfully switched to low-level mode, the motors should stop making noise.

# Run the project
To launch the demo, you first need to build and source the workspace. After that, you can connect to the Wi-Fi of the robot and switch it to low-level mode. Then, you can launch the demo using the following command :

```bash
roslaunch robotdog bringup.launch
```
> **Warning:**
> Make sure to source you workspace before launching the demo.

This launch file will start all necessary node and services to use the demo. You can now command the Unitree Go1 robot to perform action using the corresponding keys. See table below :

| Key | Action |
| -------- | ------- |
| 'u' | stand up |
| 's' | sit down |
| 'c' | crouch |
| 'q' | quit demo |

