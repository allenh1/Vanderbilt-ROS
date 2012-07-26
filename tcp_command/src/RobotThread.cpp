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
    //rostopic pub p2os_driver/MotorState cmd_motor_state -- 1.0
    cmd_publisher = nh.advertise<std_msgs::String>("/tcp_cmd", 1000);
    sim_velocity  = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 100);
    pose_listener = nh.subscribe("/pose", 10, &RobotThread::callback, this);
    start();
    return true;
}//set up the ros toys.

void RobotThread::callback(nav_msgs::Odometry msg)
{
    m_xPos = msg.pose.pose.position.x;
    m_yPos = msg.pose.pose.position.y;
    m_aPos = msg.pose.pose.orientation.w;

    ROS_INFO("Pose: (%f, %f, %f)", m_xPos, m_yPos, m_aPos);
}//callback method to update the robot's position.

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

        geometry_msgs::Twist cmd_msg;
        cmd_msg.linear.x = m_speed;
        cmd_msg.angular.z = m_angle;

        cmd_publisher.publish(msg);
        sim_velocity.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }//do ros things.
}

void RobotThread::SetSpeed(double speed, double angle)
{
    ROS_INFO("SetSpeed recieved)");
    m_speed = speed;
    m_angle = angle;
    //rostopic pub -1 /turtle1/command_velocity turtlesim/Velocity  --2.0 --0.0
}//set the speed of the robot.

void RobotThread::setCommand(QString cmd)
{
    command = cmd;
}//get a command from another thread.

void RobotThread::setPose(QList<double> to_set)
{
    if (to_set.size() > 1)
    {
        m_xPos = to_set.at(0);//x coordinate
        m_yPos = to_set.at(1);//y coordinate
    }//end if
}//end void

double RobotThread::getXSpeed(){ return m_speed; }
double RobotThread::getASpeed(){ return m_angle; }

double RobotThread::getXPos(){ return m_xPos; }
double RobotThread::getYPos(){ return m_yPos; }
double RobotThread::getAPos(){ return m_aPos; }
}//end namespace
