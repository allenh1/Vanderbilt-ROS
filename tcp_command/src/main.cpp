#include "ros/ros.h"
#include "std_msgs/String.h"
#include "Server.h"
#include <QCoreApplication>

ros::Publisher command;

void publishCommand(QString cmd)
{
    ROS_INFO("In publish function");
    std_msgs::String msg;

    std::stringstream ss;
    ss << cmd.toStdString() << "test";
    msg.data = ss.str();

    ROS_INFO("Publishing command");
    command.publish(msg);
    ROS_INFO("Published Command");
    ROS_INFO("%s", msg.data.c_str());
}

int main(int argc, char **argv)
{
    Server server;

    ros::init(argc , argv , "tcp_listener");

    ros::NodeHandle handler;

    command = handler.advertise<std_msgs::String>("/tcp_cmd", 100);

    while (ros::ok())
    {
        std_msgs::String msg;

        std::stringstream ss;
        ss << server.getCommand().toStdString() << "test";
        msg.data = ss.str();

        command.publish(msg);
    }

    return 0;
}//end main.

