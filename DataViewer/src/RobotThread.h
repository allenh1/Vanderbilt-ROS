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

    const double & getMinCircles();
    const double & getMinSegments();
    const double & getMinCurves();
    const double & getMinPoints();
    const double & getMinShapes();
    const double & getMinSegError();

    const double & getMaxCircles();
    const double & getMaxSegments();
    const double & getMaxCurves();
    const double & getMaxPoints();
    const double & getMaxShapes();
    const double & getMaxSegError();

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

    Q_SIGNAL void CircleInformation(double circle);
    Q_SIGNAL void SegmentInformation(double segment);
    Q_SIGNAL void SegmentErrorInformation(double segError);
    Q_SIGNAL void PointInformation(double point);
    Q_SIGNAL void BezierInformation(double curve);
    Q_SIGNAL void ShapeInformation(double shape);
    Q_SIGNAL void TimeInformation(double time);
private:

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

    double m_minCircleCount;
    double m_minShapeCount;
    double m_minPointCount;
    double m_minSegmentCount;
    double m_minBezierCount;
    double m_minMaxSegError;

    double m_maxCircleCount;
    double m_maxShapeCount;
    double m_maxPointCount;
    double m_maxSegmentCount;
    double m_maxBezierCount;
    double m_maxMaxSegError;

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
};
}//end namespace
#endif

