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
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <turtlesim/Velocity.h>
#include <turtlesim/Pose.h>
#include <nav_msgs/Odometry.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>

#include "ShapeAverageThread.h"

namespace data_server {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

class RobotThread : public QThread {
    Q_OBJECT
public:
    RobotThread(int argc, char **pArgv);
    virtual ~RobotThread();

    double getXPos();
    double getXSpeed();
    double getASpeed();
    double getYPos();
    double getAPos();

    const double & getCircleCount();
    const double & getTime();
    const double & getPointCount();
    const double & getSegmentCount();
    const double & getBezierCount();
    const double & getShapeCount();
    const double & getMaxSegError();

    const double & getMinCircles();
    const double & getMinSegments();
    const double & getMinCurves();
    const double & getMinPoints();
    const double & getMinShapes();
    const double & getMinSegError();

    bool init();

    //void callback(nav_msgs::Odometry msg);
    void callback(turtlesim::Pose msg);
    void dataCallback(std_msgs::String msg);
    void scanCallBack(sensor_msgs::LaserScan scan);

    void SetSpeed(double speed, double angle);
    void setPose(QList<double> to_set);
    void goToXYZ(geometry_msgs::Point goTo);
    void setCommand(QString cmd);
    void run();

    Q_SIGNAL void newCircle();
    Q_SIGNAL void newTime();
    Q_SIGNAL void newPoint();
    Q_SIGNAL void newCurve();
    Q_SIGNAL void newSegment();
    Q_SIGNAL void newShapeCount();
    Q_SIGNAL void newMaxSegError();

private:
    Q_SLOT void sendCircle();
    Q_SLOT void sendTime();
    Q_SLOT void sendSegment();
    Q_SLOT void sendPoint();
    Q_SLOT void sendCurve();
    Q_SLOT void sendShape();
    Q_SLOT void sendMaxSegError();

    void averageShapes();
    void averageSegments();
    void averageCircles();
    void averageCurves();

    double getAverage(QList<double> list);

    QString command;

    int m_Init_argc;
    char** m_pInit_argv;

    double m_speed;
    double m_angle;

    double m_circleCount;
    double m_time;
    double m_shapeCount;
    double m_pointCount;
    double m_segmentCount;
    double m_bezierCount;
    double m_maxSegError;

    double m_xPos;
    double m_yPos;
    double m_aPos;

    double m_maxRange;
    double m_minRange;

    QList<double> ranges;
    QList<double> shapeCounts;
    QList<double> circleCounts;
    QList<double> segmentCounts;
    QList<double> curveCounts;

    ros::Publisher cmd_publisher;
    ros::Publisher sim_velocity;

    ros::Subscriber pose_listener;
    ros::Subscriber data_listener;
    ros::Subscriber scan_listener;

    ShapeAverageThread m_MathThread;
};
}//end namespace
#endif

