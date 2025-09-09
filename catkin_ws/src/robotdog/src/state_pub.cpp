#include <ros/ros.h>
#include <unitree_legged_msgs/HighState.h>
#include <unitree_legged_msgs/LowState.h>
#include "unitree_legged_sdk/unitree_legged_sdk.h"
#include "convert.h"
#include <geometry_msgs/Twist.h>

using namespace UNITREE_LEGGED_SDK;
class custom
{
public:
    UDP lowUdp;
    UDP highUdp;

    HighCmd highCmd = {0};
    HighState highState = {0};

    LowCmd lowCmd = {0};
    LowState lowState = {0};

public:
    custom()
        : 
        // lowUdp(LOWLEVEL),
        lowUdp(LOWLEVEL, 8091, "192.168.123.10", 8007),
        highUdp(8090, "192.168.12.1", 8082, sizeof(HighCmd), sizeof(HighState))
    {
        highUdp.InitCmdData(highCmd);
        lowUdp.InitCmdData(lowCmd);
    }

    void highUdpSend()
    {
        // printf("high udp send is running\n");

        highUdp.SetSend(highCmd);
        highUdp.Send();
    }

    void lowUdpSend()
    {

        lowUdp.SetSend(lowCmd);
        lowUdp.Send();
    }

    void lowUdpRecv()
    {

        lowUdp.Recv();
        lowUdp.GetRecv(lowState);
    }

    void highUdpRecv()
    {
        // printf("high udp recv is running\n");

        highUdp.Recv();
        highUdp.GetRecv(highState);
    }
};

custom custom;

//ros::Subscriber subHigh;
//ros::Subscriber subLow;

ros::Publisher pubHigh;
ros::Publisher pubLow;

int main(int argc, char **argv)
{
    if (argc < 2)
    {
        std::cerr << "Usage: ros_udp [LOWLEVEL | HIGHLEVEL]" << std::endl;
        return 1;
    }

    ros::init(argc, argv, "ros_udp");

    ros::NodeHandle nh;

    if (strcasecmp(argv[1], "LOWLEVEL") == 0)
    {
        // subLow = nh.subscribe("low_cmd", 1, lowCmdCallback);
        pubLow = nh.advertise<unitree_legged_msgs::LowState>("low_state", 1);

        LoopFunc loopUdpSend("low_udp_send", 0.02, 3, [objectPtr = &custom] { objectPtr->lowUdpSend(); });
        LoopFunc loopUdpRecv("low_udp_recv", 0.02, 3, [objectPtr = &custom] { objectPtr->lowUdpRecv(); });

        loopUdpSend.start();
        loopUdpRecv.start();

        ros::Rate rate(1); // 1 Hz
        while (ros::ok())
        {
            unitree_legged_msgs::LowState lowStateRos;
            lowStateRos = state2rosMsg(custom.lowState);
            pubLow.publish(lowStateRos);
            ros::spinOnce();
            rate.sleep();
        }

        // printf("low level runing!\n");
    }
    else if (strcasecmp(argv[1], "HIGHLEVEL") == 0)
    {
        // subHigh = nh.subscribe("high_cmd", 1, highCmdCallback);
        pubHigh = nh.advertise<unitree_legged_msgs::HighState>("high_state", 1);

        LoopFunc loopUdpSend("high_udp_send", 0.02, 3, [objectPtr = &custom] { objectPtr->highUdpSend(); });
        LoopFunc loopUdpRecv("high_udp_recv", 0.02, 3, [objectPtr = &custom] { objectPtr->highUdpRecv(); });

        loopUdpSend.start();
        loopUdpRecv.start();

        ros::Rate rate(1); // 1 Hz
        while (ros::ok())
        {
            unitree_legged_msgs::HighState highStateRos;
            highStateRos = state2rosMsg(custom.highState);
            pubHigh.publish(highStateRos);
            ros::spinOnce();
            rate.sleep();
        }

        // printf("high level runing!\n");
    }
    else
    {
        std::cout << "Control level name error! Can only be highlevel or lowlevel (not case sensitive)" << std::endl;
        exit(-1);
    }

    return 0;
}