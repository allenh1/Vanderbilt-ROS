#include "RobotThread.h"

namespace server {
RobotThread::RobotThread(int argc, char** argv)
    :	init_argc(argc),
        init_argv(argv)
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
    ros::init(init_argc, init_argv, "tcp_command");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::NodeHandle nh;
    cmd_publisher = nh.advertise<std_msgs::String>("/tcp_cmd", 1000);
    start();
    return true;
}//set up the ros toys.

void RobotThread::run()
{
    ros::Rate loop_rate(1);

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

void RobotThread::SetSpeed(double speed, double angle) {
    m_Speed = speed;
    m_Angle = angle;
}

/*double RobotThread::getForwardSpeed()
{
    return m_Speed;
}

double RobotThread::getTurnSpeed()
{
    return m_Angle;
}*/

void RobotThread::setCommand(QString cmd)
{
    command = cmd;
}//get a command from another thread.

void RobotThread::EndControl() {
    m_Continue = false;
}

}//end namespace
