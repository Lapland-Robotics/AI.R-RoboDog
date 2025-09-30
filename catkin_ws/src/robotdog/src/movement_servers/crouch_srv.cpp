#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "common/waypoint.h"
#include "common/mathTypes.h"

using namespace UNITREE_LEGGED_SDK;

unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

ros::Publisher lowPub;

int count = 0;

Vec12 targetPos = (Vec12() << 0.0, 1.4, -2.67, 0.0, 1.4, -2.67, 0.0, 1.4, -2.67, 0.0, 1.4, -2.67).finished();
Waypoint targetWaypoint = Waypoint(targetPos, 1.0f);
Vec12 startPos;
float rate = 50; // Hz
float duration; //steps
float percent; //%

std::string robotState = "prone";

bool posSet = false;
bool motionInProgress = false;

void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &msg)
{
    lowState = *msg;
    if (!posSet){
        for (int i = 0; i < 12; i++){
            startPos(i) = lowState.motorState[i].q;
        }
        ROS_INFO("Initial positions set.");
        posSet = true;
    } else {
        percent += (float)1 / duration;
        if (percent > 1) percent = 1;
    }
}

void initLowCmd(){
    lowCmd.head[0] = 0xFE;
    lowCmd.head[1] = 0xEF;
    lowCmd.levelFlag = LOWLEVEL;

    for (int leg = 0; leg < 4; leg++){
        lowCmd.motorCmd[leg * 3].mode = 0x0A;  // motor switch to servo (PMSM) mode
        lowCmd.motorCmd[leg * 3].dq = 0;
        lowCmd.motorCmd[leg * 3].tau = 0;
        lowCmd.motorCmd[leg * 3].Kp = 60;
        lowCmd.motorCmd[leg * 3].Kd = 5;

        lowCmd.motorCmd[leg * 3 + 1].mode = 0x0A;  // motor switch to servo (PMSM) mode
        lowCmd.motorCmd[leg * 3 + 1].dq = 0;
        lowCmd.motorCmd[leg * 3 + 1].tau = 0;
        lowCmd.motorCmd[leg * 3 + 1].Kp = 60;
        lowCmd.motorCmd[leg * 3 + 1].Kd = 5;

        lowCmd.motorCmd[leg * 3 + 2].mode = 0x0A;  // motor switch to servo (PMSM) mode
        lowCmd.motorCmd[leg * 3 + 2].dq = 0;
        lowCmd.motorCmd[leg * 3 + 2].tau = 0;
        lowCmd.motorCmd[leg * 3 + 2].Kp = 90;
        lowCmd.motorCmd[leg * 3 + 2].Kd = 7;
    }
    ROS_INFO("Motors initialized.");
}

bool standUpCallback(__attribute__((unused)) std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (motionInProgress) {
        res.success = false;
        res.message = "Crouch motion already in progress.";
        return true;
    }
    
    ROS_INFO("Service crouch triggered.");

    posSet = false;
    percent = 0;
    motionInProgress = true;

    ros::Rate loopRate(rate);
    int timeout = 0;

    while (ros::ok() && !posSet && timeout < 100) {
        ros::spinOnce();
        loopRate.sleep();
        timeout++;
    }

    if (!posSet) {
        res.success = false;
        res.message = "Initial position not received from /low_state.";
        motionInProgress = false;
        return true;
    }

    while (ros::ok() && percent < 1.0) {
        for (int i = 0; i < 12; i++) {
                lowCmd.motorCmd[i].q = (1 - percent) * startPos(i) + percent * targetWaypoint.getMotorTargetQ(i);
        }
        lowPub.publish(lowCmd);
        ros::spinOnce();
        loopRate.sleep();
    }

    motionInProgress = false;
    ros::param::set("/robot_state", "crouch");
    res.success = true;
    res.message = "Robot crouching completed.";
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "crouch_service");
    ros::NodeHandle nh;

    lowPub = nh.advertise<unitree_legged_msgs::LowCmd>("/low_cmd", 1);
    ros::Subscriber lowSub = nh.subscribe("/low_state", 1, lowStateCallback);

    initLowCmd();
    duration = targetWaypoint.getTravelTime() * rate;

    ros::ServiceServer service = nh.advertiseService("/crouch_srv", standUpCallback);

    ROS_INFO("Service crouch ready. Waiting for calls...");
    ros::spin();
    return 0;
}
