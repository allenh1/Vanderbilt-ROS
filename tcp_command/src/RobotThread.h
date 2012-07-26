#ifndef ___ROBOTTHREAD_H___
#define ___ROBOTTHREAD_H___

#include <QThread>
#include <QObject>
#include <QStringList>
#include <stdlib.h>
#include <iostream>
#include "assert.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

namespace server {
class RobotThread : public QThread {
	Q_OBJECT
public:
    RobotThread(int argc, char **pArgv);
    virtual ~RobotThread();

    double getXPos();
    double getYPos();
    double getAPos();

    bool init();

    void callback(nav_msgs::Odometry msg);
	void SetSpeed(double speed, double angle);
    void setCommand(QString cmd);
    void run();

private:
    QString command;
	
    int m_Init_argc;
    char** m_pInit_argv;

    double m_speed;
    double m_angle;
    double m_xPos;
    double m_yPos;
    double m_aPos;

    ros::Publisher cmd_publisher;
    ros::Publisher sim_velocity;
    ros::Subscriber pose_listener;
};
}//end namespace
#endif
