# USER GUIDE

This document aim to provide support for anyone that would like to progress further in this project.

The goal of the project is to be able to use voice commands to give orders to the Unitree Go1 robot. For now, this version of the project allow using a keyboard to send a few types of command to the robot such as sit down, give paw or stand up.

This project is using ROS Melodic as a local middleware and send commands to the robot using UDP. It uses the `unitree_ros_to_real` package from Unitree Robotics [GitHub](https://github.com/unitreerobotics/unitree_ros_to_real) that provide the messages types to command the robot as well as UDP connection with the robot.

The content of this branch is an attempt to refactor the code for better kinematics and to implement reverse kinematics on the robot. The programs in this branch have not been tested on the Unitree Go1 because the robot is currently unavailable. Previously existing programs should behave the same as before.

## Command Interface

The command interface consist of 2 ROS nodes `keyboard_cmd` and `cmd_dispatcher`.

`keyboard_cmd` is a node that listen to your keyboard input and map some keys to commands to give to the robot. Each command is sent through a ROS topic `/cmd` of type `std_msgs/String`.  
See code below :  
```cpp
while (ros::ok()) {
    char c = getch();
    if (c == 0) continue; // No input
    if (c == 'u') msg.data = "stand_up";
    else if (c == 's') msg.data = "sit";
    else if (c == 'c') msg.data = "crouch";
    else if (c == 'p') msg.data = "give_paw";
    else if (c == 'r') msg.data = "roll";
    else if (c == 'q') break;
    else continue;

    pub.publish(msg);
}
``` 

Another node can easily replace this one by sending strings on the same topic while using another type of input, like text from speech of remote controller.

`cmd_dispatcher` is a node that listen to the `/cmd` and map some strings to ROS services. For now, the node only uses services of type `std_srvs/Trigger` but other types of services can surely be used.  
See code below :  
```cpp
if (msg->data == "stand_up") {
    ROS_INFO("Command: stand_up");
    success = standUpClient.call(srv);
} else if (msg->data == "sit") {
    ROS_INFO("Command: sit");
    success = sitClient.call(srv);
} else if (msg->data == "crouch") {
    ROS_INFO("Command: crouch");
    success = crouchClient.call(srv);
} else if (msg->data == "give_paw") {
    ROS_INFO("Command: give_paw");
    success = givePawClient.call(srv);
} else {
    ROS_WARN("Command unknown: %s", msg->data.c_str());
    return;
}
```

## UDP Interface

The UDP connection to the robot is handled by the `udp_ros` node. It is based on the `ros_udp.cpp` program from the `unitree_ros_to_real` [GitHub](https://github.com/unitreerobotics/unitree_ros_to_real) package from Unitree Robotics. We made some modifications to it to add reconnection on failure for better stability and reliability.

This node takes as argument `lowlevel` or `highlevel` to select which type of command to send to the Unitree Go1. Lowlevel is for joint level control (motor by motor) and highlevel is to give the same type of command as the remote controller. This project uses lowlevel control. 

This node allows us to receive the state of the robot and publish it on topics `low_state` or `high_state` depending on the types of commands, and listen to the topics `low_cmd` or `high_cmd` and send them to the Unitree Go1 robot. It also sends a heartbeat message to the robot at a lower rate (this heartbeat can probably be deleted).

We also provide the `udp_listen` and `high_state_logger` nodes. The first node publishes the state of the robot without sending the commands. The second one subscribe to the `high_state` topic and save the state of the robot in a csv file for later use. These nodes are mainly used to save the state of the robot when performing highlevel actions and try to reproduce them.

## Commands the robot

There are a few nodes used to publish the lowlevel commands to the UnitreeGo1. The nodes `stand_up_srv`, `crouch_srv`, `sit_srv`, `give_paw_srv` and `roll_srv` are services nodes triggered by the `cmd_dispatcher` node. `roll_srv` Have not been tested yet on the robot

All present services use the same structure, they have an initialisation part, a service callback that execute when the service is triggered, and a topic callback executed when a lowstate is received. For the kinematic part, it uses point-to-point forward kinamatic. Point-to-point means that the robot will try to reach a given position from it's starting position. Forward kinematics means that the calculus is made using motors angles, where reverse kinematics would use position of the foots of the robot. The programm use a linear approach to calculate a set of commands between the starting point and the target point.

Each position is a set of twelve float that correspond to the angle of the twelves motors. For each leg in order : Front Right, Front Left, Rear Right and Rear Left; position are given in order : Hip, Thigh, Calf. See [This link](https://github.com/unitreerobotics/unitree_legged_sdk/blob/acc36dfd48755f5a9889ef3cc9ee31604ebfe41c/include/unitree_legged_sdk/go1_const.h) for min and max values of each joint. 

To implement kinematic, we provide the `waypoint` class that store a position of the robot and the time to reach it from the starting or previous position. We also provide the `trajectory` class that store a list of waypoints to create a trajectory to follow. The file `kinematic.h` also provide fonctions to convert foot poses to motors angle for inverse kinematic. These functions come from the unitree_guide [GitHub](https://github.com/unitreerobotics/unitree_guide) package from Unitree Robotics.

The main function of the nodes usually look like this :  
```cpp
int main(int argc, char **argv)
{
    ros::init(argc, argv, "give_paw_service");
    ros::NodeHandle nh;

    lowPub = nh.advertise<unitree_legged_msgs::LowCmd>("/low_cmd", 1);
    ros::Subscriber lowSub = nh.subscribe("/low_state", 1, lowStateCallback);
    sitClient = nh.serviceClient<std_srvs::Trigger>("/sit_srv");

    initLowCmd();
    duration = targetWaypoint.getTravelTime() * rate;

    ros::ServiceServer service = nh.advertiseService("/give_paw_srv", givePawCallback);

    ROS_INFO("Service give_paw ready. Waiting for calls...");
    ros::spin();
    return 0;
}
```
This function initialises the ROS node, subscribe to the `low_state` topics to receive robot state, advertise the `low_cmd` to send commands. It also advertises the service provided by the node, and possibly subscribe to another service if needed (in the case of `give_paw_srv`). The `initLowCmd` function is used to set the control mode of the motors (servo mode) and the coefficients of the PD regulator of each motor.  

The `duration` variable is the number of steps (or number of command) to go from the starting point to the target point, according to the duration of the movement and the command publish rate. 

The callback triggered when a state is received is the following :
```cpp
void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &msg)
{
    lowState = *msg;
    if (!posSet){   
        for (int i = 0; i < 12; i++){
            startPos(i) = lowState.motorState[i].q;
        }
        ROS_INFO("Initial positions set.");
        posSet = true;
    } else if( robotState != "sit" ){
        percent += (float)1 / duration;
        if (percent > 1) percent = 1;
    }
}
```

This function has two roles. First, it set the initial position of the robot that need to be known to command the robot. Second is to update the `percent` variable. This variable symbolises how much of the movement have been done. The evolution of this variable is done in this callback to ensure that the UDP connection with the robot is still working and avoid unwanted behaviour.

Finally, the service callback contains all the movement logic of the robot. It uses a ROS param named `robot_state` to share the state of the robot to other nodes.  
See below a commented version of the program for better understanding of what it does :
```cpp
bool givePawCallback(__attribute__((unused)) std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    // If motion already in progress (service triggered two times), stop the second call
    if (motionInProgress) {
        res.success = false;
        res.message = "Give paw motion already in progress.";
        return true;
    }

    // Give_paw service need the robot to sit down first
    // If robot not already sit, call the service to sit 
    ros::param::get("/robot_state", robotState);
    if (robotState != "sit") {
        ROS_INFO("Robot not sitting. Calling sit service first...");
        std_srvs::Trigger sitSrv;
        if (!sitClient.call(sitSrv) || !sitSrv.response.success) {
            res.success = false;
            res.message = "Failed to sit before giving paw.";
            return true;
        }
        robotState = "sit";
        ros::param::set("/robot_state", "sit");
    }

    ROS_INFO("Service give paw triggered.");

    // Init movement
    posSet = false;
    percent = 0;
    motionInProgress = true;

    ros::Rate loopRate(rate);
    int timeout = 0;

    // Lowstate timeout
    while (ros::ok() && !posSet && timeout < 100) {
        ros::spinOnce();
        loopRate.sleep();
        timeout++;
    }

    // If timeout, give up movement 
    if (!posSet) {
        res.success = false;
        res.message = "Initial position not received from /low_state.";
        motionInProgress = false;
        return true;
    }

    // Else, proceed to movement
    // The percent variable is update by the lowstate callback
    while (ros::ok() && percent < 1.0) {
        // For each motors
        for (int i = 0; i < 12; i++) {
                // Interpolate commands between start point and target point (linear)
                lowCmd.motorCmd[i].q = (1 - percent) * startPos(i) + percent * targetWaypoint.getMotorTargetQ(i);
        }
        lowPub.publish(lowCmd);
        ros::spinOnce();
        loopRate.sleep();
    }

    // When movement finished, respond to the caller of the service and set the state of the robot
    motionInProgress = false;
    ros::param::set("/robot_state", "give_paw");
    res.success = true;
    res.message = "Robot give paw completed.";
    return true;
}
```

In the case of `roll_srv`, we use a trajectory of fifteen waypoints as a trajectory. In this case, the movement logic is modified to handle each waypoint of the trajectory in order.
See code below :
```cpp
ros::param::set("/robot_state", "rolling");
for (int wp = 0; wp < traj.size(); wp++ && ros::ok()) {
    targetPos = traj[wp].getTargetQ();
    duration = traj[wp].getTravelTime() * rate;
    percent = 0.0;
    while (ros::ok() && percent < 1.0) {
        for (int j = 0; j < 12; j++) {
            lowCmd.motorCmd[j].q = (1 - percent) * startPos(j) + percent * targetPos(j);
        }
        lowPub.publish(lowCmd);
        ros::spinOnce();
        loopRate.sleep();
    }
    startPos = targetPos;
}
```

## Launch

This project contains two launch file. The first one, `bringup.launch`, allows us to launch commands interfaces, udp interface in lowlevel mode and all services at the same time to control the robot using the keyboard. The second one, `high_state_logger.launch`, is used to record state of the robot performing a highlevel movement for later use. 

## Using a record csv

`high_state_plot.m` is a Matlab program to plot the state of the robot on graphics from a csv file recorder with the `high_state_logger` node. It uses a set of flags to select which plots are shown. User can also use `plot_limit` to reduce the plot to useful samples, and can set via points, using sample numbers, that will be shown on plots as vertical lines. The `out` variable will also contain the position of each motors for the set via points and the time between the start of the movement and the time when each via point is reached.