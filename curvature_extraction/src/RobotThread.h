#ifndef ___ROBOTTHREAD_H___
#define ___ROBOTTHREAD_H___

#include <QThread>
#include <QObject>
#include <QStringList>
#include <QFile>
#include <QTextStream>
#include <stdlib.h>
#include <iostream>
#include "assert.h"

#include <ros/ros.h>
#include <ros/network.h>
#include <sensor_msgs/LaserScan.h>
#include <nav_msgs/Odometry.h>
#include "Shape.h"

namespace data_server {

struct LaserPoint
{
    double range;
    double bearing;

    double px;
    double py;
};

class RobotThread : public QThread {
    Q_OBJECT
public:
    RobotThread(int argc, char **pArgv);
    virtual ~RobotThread();

    bool init();

    //void callback(nav_msgs::Odometry msg);
    void scanCallBack(sensor_msgs::LaserScan scan);
    void callback(nav_msgs::Odometry msg);
    void run();


private:
    int m_Init_argc;
    char** m_pInit_argv;

    bool m_isFirstMessage;

    QList<LaserPoint> m_scanData;
    void extractCurves(QList<LaserPoint> scan_data);

    QList<Shape> m_shapes;

    double m_xPos;
    double m_yPos;
    double m_aPos;

    ros::Subscriber scan_listener;
    ros::Subscriber pose_listener;

    ros::Publisher cloud_pcl;
};
}//end namespace
#endif

