#include "RobotThread.h"

namespace server {
RobotThread::RobotThread(int argc, char** pArgv)
    :	m_Init_argc(argc),
        m_pInit_argv(pArgv)
{}

RobotThread::~RobotThread()
{
    if (ros::isStarted())
    {
        ros::shutdown();
        ros::waitForShutdown();
    }//end if

    wait();
}//end destructor

bool RobotThread::init()
{
    ros::init(m_Init_argc, m_pInit_argv, "tcp_command");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;
    cmd_publisher = nh.advertise<std_msgs::String>("/tcp_cmd", 1000);
    start();
    return true;
}//set up the ros toys.

void RobotThread::run()
{
    ros::Rate loop_rate(1);
    command = "empty";
    while (ros::ok())
    {
        std_msgs::String msg;
        std::stringstream ss;
        ss << command.toStdString();

        msg.data = ss.str();
        cmd_publisher.publish(msg);
        ros::spinOnce();
        loop_rate.sleep();
    }//do ros things.
}

void RobotThread::setCommand(QString cmd)
{
    command = cmd;
}//get a command from another thread.

}//end namespace
