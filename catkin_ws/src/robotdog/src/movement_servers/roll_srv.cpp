#include "ros/ros.h"
#include "std_srvs/Trigger.h"
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "common/waypoint.h"
#include "common/trajectory.h"
#include "traj/roll.h"

using namespace UNITREE_LEGGED_SDK;

unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

ros::Publisher lowPub;
ros::ServiceClient standUpClient;

Trajectory traj = get_roll_traj();
Vec12 targetPos;
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

bool rollCallback(__attribute__((unused)) std_srvs::Trigger::Request &req, std_srvs::Trigger::Response &res)
{
    if (motionInProgress) {
        res.success = false;
        res.message = "Roll motion already in progress.";
        return true;
    }

    ros::param::get("/robot_state", robotState);
    if (robotState != "stand_up") {
        ROS_INFO("Robot not standing up. Calling stand_up service first...");
        std_srvs::Trigger standUpSrv;
        if (!standUpClient.call(standUpSrv) || !standUpSrv.response.success) {
            res.success = false;
            res.message = "Failed to stand up before giving paw.";
            return true;
        }
        ros::param::set("/robot_state", "stand_up");
    }

    ROS_INFO("Service roll triggered.");

    percent = 0.0;
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

    motionInProgress = false;
    posSet = false;
    ros::param::set("/robot_state", "stand_up");
    res.success = true;
    res.message = "Robot rolling completed.";
    ROS_INFO("Roll motion completed successfully.");
    return true;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "roll_service");
    ros::NodeHandle nh;

    lowPub = nh.advertise<unitree_legged_msgs::LowCmd>("/low_cmd", 1);
    ros::Subscriber lowSub = nh.subscribe("/low_state", 1, lowStateCallback);
    standUpClient = nh.serviceClient<std_srvs::Trigger>("/stand_up_srv");

    initLowCmd();

    ros::ServiceServer service = nh.advertiseService("/roll_srv", rollCallback);

    ROS_INFO("Service roll ready. Waiting for calls...");
    ros::spin();
    return 0;
}
