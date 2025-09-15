#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

ros::Publisher lowPub;
ros::ServiceClient sitClient;

int count = 0;

float targetPos[12] = {0.3, 0.0, -1.8, // paw
                        -0.4, 1.3, -1.0,
                        -0.4, 1.7, -2.5,
                        0.4, 1.7, -2.5};
float startPos[12];
float rate = 50;
float duration = 0.5*rate;   //steps
float percent = 0;       //%

std::string robotState = "prone";

bool posSet = false;
__attribute__((unused)) bool motionInProgress = false;

void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &msg)
{
    lowState = *msg;
    if (!posSet){
        for (int i = 0; i < 12; i++)
        {
            startPos[i] = lowState.motorState[i].q;
            lowCmd.motorCmd[i].q = startPos[i];
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

bool givePawCallback(__attribute__((unused)) std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (motionInProgress) {
        res.success = false;
        res.message = "Give paw motion already in progress.";
        return true;
    }

    ros::param::get("/robot_state", robotState);
    if (robotState != "sit") {
        ROS_INFO("Robot not sitting. Calling sit service first...");
        std_srvs::Trigger sitSrv;
        if (!sitClient.call(sitSrv) || !sitSrv.response.success) {
            res.success = false;
            res.message = "Failed to sit before giving paw.";
            return true;
        }
        ros::param::set("/robot_state", "sit");
    }

    ROS_INFO("Service give paw triggered.");

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
        for (int j = 0; j < 12; j++) {
            lowCmd.motorCmd[j].q = (1 - percent) * startPos[j] + percent * targetPos[j];
        }
        lowPub.publish(lowCmd);
        ros::spinOnce();
        loopRate.sleep();
    }

    motionInProgress = false;
    ros::param::set("/robot_state", "give_paw");
    res.success = true;
    res.message = "Robot give paw completed.";
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "give_paw_service");
    ros::NodeHandle nh;

    lowPub = nh.advertise<unitree_legged_msgs::LowCmd>("/low_cmd", 1);
    ros::Subscriber lowSub = nh.subscribe("/low_state", 1, lowStateCallback);
    sitClient = nh.serviceClient<std_srvs::Trigger>("/sit_srv");

    initLowCmd();

    ros::ServiceServer service = nh.advertiseService("/give_paw_srv", givePawCallback);

    ROS_INFO("Service give_paw ready. Waiting for calls...");
    ros::spin();
    return 0;
}
