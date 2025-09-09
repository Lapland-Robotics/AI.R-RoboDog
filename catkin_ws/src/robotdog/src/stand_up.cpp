#include <ros/ros.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

unitree_legged_msgs::LowCmd lowCmd;
unitree_legged_msgs::LowState lowState;

int count = 0;

float targetPos[12] = {0.0, 0.8, -1.6,
                        0.0, 0.8, -1.6, 
                        0.0, 0.8, -1.6, 
                        0.0, 0.8, -1.6};
float startPos[12];
float rate = 50;
float duration = 2 * rate;   //steps
float percent = 0;       //%

bool posSet = false;

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
        percent += (float)1/duration;
        percent = percent > 1 ? 1 : percent;
    }
}

void init(){
    lowCmd.head[0] = 0xFE;
    lowCmd.head[1] = 0xEF;
    lowCmd.levelFlag = LOWLEVEL;
    // motors initialisation
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

int main(int argc, char **argv)
{
    ros::init(argc, argv, "example_standing_without_lcm");

    std::cout << "Communication level is set to LOW-level." << std::endl
              << "WARNING: Make sure the robot is hung up." << std::endl
              << "Press Enter to continue..." << std::endl;
    std::cin.ignore();
    
    ros::NodeHandle nh;
    ros::Rate loop_rate(rate);

    ros::Publisher low_pub = nh.advertise<unitree_legged_msgs::LowCmd>("low_cmd", 1);
    ros::Subscriber low_sub = nh.subscribe("low_state", 1, lowStateCallback);

    init();

    while (ros::ok())
    {
        ros::spinOnce();

        if (!posSet) {
            ROS_INFO("Waiting for low_state message...");
            loop_rate.sleep();
            continue;  // Skip until initial position is set
        }

        // ROS_INFO("Publishing low_cmd...");
        for(int j=0; j<12; j++){
            lowCmd.motorCmd[j].q = (1 - percent) * startPos[j] + percent * targetPos[j];
        }

        low_pub.publish(lowCmd);
        loop_rate.sleep();
    }

    return 0;
}