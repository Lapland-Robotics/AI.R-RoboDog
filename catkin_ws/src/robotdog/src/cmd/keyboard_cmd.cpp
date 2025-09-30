#include "ros/ros.h"
#include "std_msgs/String.h"
#include <termios.h>
#include <unistd.h>
#include <iostream>

char getch()
{
    char buf = 0;
    struct termios old = {0};
    if (tcgetattr(0, &old) < 0) perror("tcgetattr()");
    old.c_lflag &= ~ICANON;
    old.c_lflag &= ~ECHO;
    old.c_cc[VMIN] = 0;
    old.c_cc[VTIME] = 0;
    if (tcsetattr(0, TCSANOW, &old) < 0) perror("tcsetattr ICANON");

    fd_set set;
    struct timeval timeout;
    FD_ZERO(&set);
    FD_SET(0, &set);
    timeout.tv_sec = 0;
    timeout.tv_usec = 100000; // 100 ms

    int rv = select(1, &set, NULL, NULL, &timeout);
    if (rv > 0) {
        read(0, &buf, 1);
    }

    old.c_lflag |= ICANON;
    old.c_lflag |= ECHO;
    if (tcsetattr(0, TCSADRAIN, &old) < 0) perror("tcsetattr ~ICANON");

    return buf;
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "keyboard_cmd_node");
    ros::NodeHandle nh;

    ros::Publisher pub = nh.advertise<std_msgs::String>("/cmd", 10);

    std_msgs::String msg;
    ROS_INFO("Press u=stand_up, s=sit, c=crouch, p=give paw, r=roll, q=quit");

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

    return 0;
}
