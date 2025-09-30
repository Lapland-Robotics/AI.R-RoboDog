#include <ros/ros.h>
#include <ros/package.h>
#include <fstream>
#include <string>
#include <iomanip>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/MotorCmd.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"

using namespace UNITREE_LEGGED_SDK;

std::ofstream dataFile;

int motorIndices[12] = {
    FR_0, FR_1, FR_2,
    FL_0, FL_1, FL_2,
    RR_0, RR_1, RR_2,
    RL_0, RL_1, RL_2
};

void highStateCallback(const unitree_legged_msgs::HighState::ConstPtr& msg)
{
    ros::Time t = ros::Time::now();
    double timestamp = t.toSec();

    dataFile << std::fixed << std::setprecision(6) << timestamp << ";";

    // Log motor positions, speeds, and torques
    for (int i = 0; i < 12; ++i)
    {
        int idx = motorIndices[i];
        const auto& motor = msg->motorState[idx];

        dataFile << std::fixed << std::setprecision(6) << motor.q << ";"
                 << std::fixed << std::setprecision(6) << motor.dq << ";"
                 << std::fixed << std::setprecision(6) << motor.tauEst << ";";
    }
    // Log foot forces
    for (int i = 0; i < 4; ++i)
    {
        dataFile << msg->footForce[i] << ";";
    }
    // Log foot positions relative to body
    for (int i = 0; i < 4; ++i)
    {
        const auto& pos = msg->footPosition2Body[i];
        dataFile << std::fixed << std::setprecision(6) << pos.x << ";"
                 << std::fixed << std::setprecision(6) << pos.y << ";"
                 << std::fixed << std::setprecision(6) << pos.z << (i < 3 ? ";" : "\n");
    }

}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "high_state_logger");
    ros::NodeHandle nh("~"); // use private namespace for params

    // Get package path
    std::string pkgPath = ros::package::getPath("robotdog");

    // Get filename from parameter or set default
    std::string filename;
    nh.param<std::string>("output_file", filename, "motor_data.csv");

    // Full path
    std::string fullPath = pkgPath + "/datas/" + filename;

    // Create directory if needed
    system(("mkdir -p " + pkgPath + "/datas").c_str());

    // Open CSV file
    dataFile.open(fullPath, std::ios::out | std::ios::trunc); // trunc clears old data
    if (!dataFile.is_open()) {
        ROS_ERROR("Failed to open file: %s", fullPath.c_str());
        return 1;
    }

    ROS_INFO("Saving motor data to: %s", fullPath.c_str());
    dataFile << "timestamp";
    // for motor position, speed and torque
    for (int i = 0; i < 12; ++i)
    {
        dataFile << ";q" << i << ";dq" << i << ";tau" << i;
    }
    // for foot forces
    for (int i = 0; i < 4; ++i)
    {
        dataFile << ";ff" << i;
    }
    // for foot position to body
    for (int i = 0; i < 4; ++i)
    {
        dataFile << ";px" << i << ";py" << i << ";pz" << i;
    }
    dataFile << "\n";

    ros::Subscriber sub = nh.subscribe("/high_state", 10, highStateCallback);
    ros::spin();

    dataFile.close();
    return 0;
}
