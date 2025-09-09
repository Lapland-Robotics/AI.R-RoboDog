#include <unitree_legged_msgs/LowState.h>
#include <unitree_legged_msgs/HighState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include <ros/ros.h>

using namespace UNITREE_LEGGED_SDK;

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr &msg)
{
    printf("FR_0_pos = %f\n", msg->motorState[FR_0].q);
    printf("FR_1_pos = %f\n", msg->motorState[FR_1].q);
    printf("FR_2_pos = %f\n", msg->motorState[FR_2].q);
    printf("FL_0_pos = %f\n", msg->motorState[FL_0].q);
    printf("FL_1_pos = %f\n", msg->motorState[FL_1].q);
    printf("FL_2_pos = %f\n", msg->motorState[FL_2].q);
    printf("RR_0_pos = %f\n", msg->motorState[RR_0].q);
    printf("RR_1_pos = %f\n", msg->motorState[RR_1].q);
    printf("RR_2_pos = %f\n", msg->motorState[RR_2].q);
    printf("RL_0_pos = %f\n", msg->motorState[RL_0].q);
    printf("RL_1_pos = %f\n", msg->motorState[RL_1].q);
    printf("RK_2_pos = %f\n", msg->motorState[RL_2].q);
    printf("\n");
}

void lowStateCallback(const unitree_legged_msgs::LowState::ConstPtr &msg)
{   
    printf("FR_0_pos = %f\n", msg->motorState[FR_0].q);
    printf("FR_1_pos = %f\n", msg->motorState[FR_1].q);
    printf("FR_2_pos = %f\n", msg->motorState[FR_2].q);
    printf("FL_0_pos = %f\n", msg->motorState[FL_0].q);
    printf("FL_1_pos = %f\n", msg->motorState[FL_1].q);
    printf("FL_2_pos = %f\n", msg->motorState[FL_2].q);
    printf("RR_0_pos = %f\n", msg->motorState[RR_0].q);
    printf("RR_1_pos = %f\n", msg->motorState[RR_1].q);
    printf("RR_2_pos = %f\n", msg->motorState[RR_2].q);
    printf("RL_0_pos = %f\n", msg->motorState[RL_0].q);
    printf("RL_1_pos = %f\n", msg->motorState[RL_1].q);
    printf("RK_2_pos = %f\n", msg->motorState[RL_2].q);
    printf("\n");
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "node_high_state_sub");

    ros::NodeHandle nh;

    unitree_legged_msgs::HighState highStateRos;

    ros::Subscriber highSub = nh.subscribe("high_state", 1, highStateCallback);
    ros::Subscriber lowSub = nh.subscribe("low_state", 1, lowStateCallback);

    ros::spin();

    return 0;
}