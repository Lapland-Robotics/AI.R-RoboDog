#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"

ros::ServiceClient standUpClient;
ros::ServiceClient sitClient;
ros::ServiceClient crouchClient;
ros::ServiceClient givePawClient;
ros::ServiceClient rollClient;

void cmdCallback(const std_msgs::String::ConstPtr& msg)
{
    std_srvs::Trigger srv;
    bool success = false;

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
    } else if (msg->data == "roll") {
        ROS_INFO("Command: roll");
        success = rollClient.call(srv);
    } else {
        ROS_WARN("Command unknown: %s", msg->data.c_str());
        return;
    }

    if (success) {
        if (srv.response.success) {
            ROS_INFO("Response: %s", srv.response.message.c_str());
        } else {
            ROS_WARN("Service called returned failure: %s", srv.response.message.c_str());
        }
    } else {
        ROS_ERROR("Failed to call service for command: %s", msg->data.c_str());
    }
}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "command_dispatcher");
    ros::NodeHandle nh;

    standUpClient  = nh.serviceClient<std_srvs::Trigger>("/stand_up_srv");
    sitClient      = nh.serviceClient<std_srvs::Trigger>("/sit_srv");
    crouchClient   = nh.serviceClient<std_srvs::Trigger>("/crouch_srv");
    givePawClient  = nh.serviceClient<std_srvs::Trigger>("/give_paw_srv");
    rollClient     = nh.serviceClient<std_srvs::Trigger>("/roll_srv");

    // Wait for services to be available
    ROS_INFO("Waiting for services...");
    standUpClient.waitForExistence();
    sitClient.waitForExistence();
    crouchClient.waitForExistence();
    givePawClient.waitForExistence();
    rollClient.waitForExistence();
    ROS_INFO("All services are available.");

    ros::Subscriber sub = nh.subscribe("/cmd", 10, cmdCallback);

    ros::spin();
    return 0;
}
