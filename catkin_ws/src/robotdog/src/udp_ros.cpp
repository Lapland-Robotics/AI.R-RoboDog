#include <ros/ros.h>
#include <unitree_legged_msgs/HighCmd.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowCmd.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"

using namespace UNITREE_LEGGED_SDK;

ros::Publisher pubHigh;
ros::Publisher pubLow;

constexpr int MAX_MISSES = 50;

class custom
{
public:
    UDP lowUdp;
    UDP highUdp;

    HighCmd highCmd = {0};
    HighState highState = {0};

    LowCmd lowCmd = {0};
    LowState lowState = {0};

    int lowMisses = 0;
    int highMisses = 0;

public:
    custom()
        : lowUdp(LOWLEVEL, 8091, "192.168.123.10", 8007),
          highUdp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        highUdp.InitCmdData(highCmd);
        lowUdp.InitCmdData(lowCmd);
    }

    void highUdpSend() {
        highUdp.SetSend(highCmd);
        highUdp.Send();
    }

    void lowUdpSend() {
        lowUdp.SetSend(lowCmd);
        lowUdp.Send();
    }

    void lowUdpRecv() {
        if (lowUdp.Recv() != -1) {
            lowUdp.GetRecv(lowState);
            unitree_legged_msgs::LowState lowStateRos = state2rosMsg(lowState);
            pubLow.publish(lowStateRos);
            lowMisses = 0;
        } else {
            lowMisses++;
            if (lowMisses > MAX_MISSES) {
                ROS_WARN("Low UDP lost. Reconnecting...");
                lowUdp.~UDP();
                new(&lowUdp) UDP(LOWLEVEL, 8091, "192.168.123.10", 8007);
                lowUdp.InitCmdData(lowCmd);
                lowMisses = 0;
            }
        }
    }

    void highUdpRecv() {
        if (highUdp.Recv() != -1) {
            highUdp.GetRecv(highState);
            unitree_legged_msgs::HighState highStateRos = state2rosMsg(highState);
            pubHigh.publish(highStateRos);
            highMisses = 0;
        } else {
            highMisses++;
            if (highMisses > MAX_MISSES) {
                ROS_WARN("High UDP lost. Reconnecting...");
                highUdp.~UDP();
                new(&highUdp) UDP(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState));
                highUdp.InitCmdData(highCmd);
                highMisses = 0;
            }
        }
    }

    void sendHeartbeat() {
        lowUdp.SetSend(lowCmd);
        lowUdp.Send();
        highUdp.SetSend(highCmd);
        highUdp.Send();
    }
};

custom custom;

void highCmdCallback(const unitree_legged_msgs::HighCmd::ConstPtr &msg)
{
    custom.highCmd = rosMsg2Cmd(msg);
}

void lowCmdCallback(const unitree_legged_msgs::LowCmd::ConstPtr &msg)
{
    custom.lowCmd = rosMsg2Cmd(msg);
}

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: rosrun robotdog udp_pub_sub_node <LOWLEVEL|HIGHLEVEL>" << std::endl;
        return -1;
    }

    ros::init(argc, argv, "ros_udp");
    ros::NodeHandle nh;

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        ros::Subscriber subLow = nh.subscribe("low_cmd", 1, lowCmdCallback);
        pubLow = nh.advertise<unitree_legged_msgs::LowState>("low_state", 1);

        LoopFunc loopUdpSend("low_udp_send", 0.002, 3, [objectPtr = &custom] { objectPtr->lowUdpSend(); });
        LoopFunc loopUdpRecv("low_udp_recv", 0.002, 3, [objectPtr = &custom] { objectPtr->lowUdpRecv(); });
        LoopFunc loopHeartbeat("heartbeat", 0.02, 3, [objectPtr = &custom] { objectPtr->sendHeartbeat(); });

        loopUdpSend.start();
        loopUdpRecv.start();
        loopHeartbeat.start();

        ros::spin();
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        ros::Subscriber subHigh = nh.subscribe("high_cmd", 1, highCmdCallback);
        pubHigh = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);

        LoopFunc loopUdpSend("high_udp_send", 0.002, 3, [objectPtr = &custom] { objectPtr->highUdpSend(); });
        LoopFunc loopUdpRecv("high_udp_recv", 0.002, 3, [objectPtr = &custom] { objectPtr->highUdpRecv(); });
        LoopFunc loopHeartbeat("heartbeat", 0.02, 3, [objectPtr = &custom] { objectPtr->sendHeartbeat(); });

        loopUdpSend.start();
        loopUdpRecv.start();
        loopHeartbeat.start();

        ros::spin();
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel(not case sensitive)" << std::endl;
        exit(-1);
    }

    return 0;
}
