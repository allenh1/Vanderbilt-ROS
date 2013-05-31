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
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>

#include "ShapeAverageThread.h"

namespace data_server {

struct LaserData
{
    QList<double> m_ranges;

    double m_variance;
    double m_mean;
};

class RobotThread : public QThread {
    Q_OBJECT
public:
    RobotThread(int argc, char **pArgv);
    virtual ~RobotThread();

    bool init();

    //void callback(nav_msgs::Odometry msg);
    void scanCallBack(sensor_msgs::LaserScan scan);

    void run();

    Q_SLOT void saveData();
private:
    int m_Init_argc;
    char** m_pInit_argv;

    bool m_isFirstMessage;

    QList<LaserData> m_pastData;

    ros::Subscriber scan_listener;
};
}//end namespace
#endif

