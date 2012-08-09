#ifndef ___ROBOTTHREAD_H___
#define ___ROBOTTHREAD_H___

#include <QThread>
#include <QObject>
#include <QStringList>
#include <stdlib.h>
#include <QGenericMatrix>
#include <iostream>
#include "assert.h"
#include <QMutex>

#include <ros/ros.h>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/Core>
#include <Eigen/StdVector>
#include <ros/network.h>
#include <std_msgs/String.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/OccupancyGrid.h>
#include <actionlib/client/simple_action_client.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <QGenericMatrix>

namespace server {

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef Eigen::Matrix<double, 2, 3> Matrix2x3d;//Matrix, 2 rows, 3 cols, type: double
typedef Eigen::Matrix<double, 2, 2> Matrix2x2d;//Matrix, 2 rows, 2 cols, type: double
typedef Eigen::Matrix<double, 6, 6> Matrix6x6d;
typedef Eigen::Matrix<double, 3, 3> Matrix3x3d;

const double PI = 3.141592653589793238462643383279;
const double Delta_x = 0;//x of laser wrt robot
const double Delta_y = 0.1778;//y of laser wrt robot
const double dcritp = 0.3;
const double dcritl = 0.3;
const double alpha_crit = PI / 6;

struct globalPoint
{
    double m_x;
    double m_y;
    double m_rho;
    double m_theta;

    Matrix2x2d Cp; //covariance
    Matrix2x2d Cs; //covariance
    Matrix3x3d Cxr; //covariance

    Matrix2x3d F;//Jacobian
    Matrix2x2d G;//Jacobian
};//structure for a laser point.

struct V
{
    double m_m;
    double m_q;
};

struct segment
{
    double m_length;
    double Rx;
    double Ry;
    double Rxx;
    double Ryy;
    double Rxy;
    double N1;
    double N2;
    double T;
    double m;
    double q;
    double s;
    double t;

    V line;

    Matrix2x2d Cv;
    Matrix2x2d J;

    std::vector<globalPoint, Eigen::aligned_allocator<globalPoint> > m_points;
};

inline double getAverage(int a, int b, QList<double> list)
{
    double sum = 0;

    for (int x = a; x < b; ++x)
        sum += list.at(x);

    return sum / (b - a);
}//get the average of the above.

inline double stdDev(QList<double> list)
{
    double mean = getAverage(0, list.size() - 1, list);
    double sum  = 0;

    for (int x = 0; x < list.size(); x++)
        sum += pow(list.at(x) - mean, 2);

    return sqrt(sum / (list.size() - 1));
}

inline double getDistance(globalPoint p1, globalPoint p2)
{ return sqrt(pow(p2.m_x - p1.m_x, 2) + pow(p2.m_y - p1.m_y, 2)); }

inline double getMaxError(segment s1)
{ return s1.m_points.at(s1.m_points.size() - 1).m_x * 0.3 - s1.m_points.at(0).m_x * .3; }

inline double getLineError(segment s1, segment s2)
{
    if (getDistance(s1.m_points.at(0), s1.m_points.at(0)) > s1.m_length)
        return getMaxError(s1);
    if (s1.m_length > s2.m_length)
    {
        double x1 = s1.m_points.at(s1.m_points.size() - 1).m_x;
        double x2 = s1.m_points.at(0).m_x;
        double delta_m = sqrt(pow(s1.m - s2.m, 2));
        double delta_q = sqrt(pow(s1.q - s2.q, 2));

        double upper = (delta_m * pow(x1, 2)) / 2.0 + delta_q * x1;
        double lower = (delta_m * pow(x2, 2)) / 2.0 + delta_q * x2;

        return sqrt(pow(upper - lower, 2));
    }//end if.

    else
    {
        double x1 = s2.m_points.at(s2.m_points.size() - 1).m_x;
        double x2 = s2.m_points.at(0).m_x;
        double delta_m = sqrt(pow(s2.m - s1.m, 2));
        double delta_q = sqrt(pow(s2.q - s1.q, 2));

        double upper = (delta_m * pow(x1, 2)) / 2.0 + delta_q * x1;
        double lower = (delta_m * pow(x2, 2)) / 2.0 + delta_q * x2;

        return sqrt(pow(upper - lower, 2));
    }//end else
}//get the error between segments

inline double summation(QList<double> toSum)
{
    double sum = 0;

    for (int x = 0; x < toSum.size(); x++)
        sum += toSum.at(x);

    return sum;
}//sum all members of the list

inline QList<double> raiseListToPow(QList<double> list, int exponent)
{
    QList<double> toReturn;

    for (int x = 0; x < list.size(); x++)
        toReturn.push_back(pow(list.at(x), exponent));

    return toReturn;
}//raise all elements of a QList to a power and return.

inline QList<double> multiplyIndecies(QList<double> list1, QList<double> list2)
{
    QList<double> toReturn;

    for (int x = 0; x < list1.size(); x++)
        toReturn.push_back(list1.at(x) * list2.at(x));

    return toReturn;
}

inline double getAngleDif(globalPoint p1, globalPoint p2, globalPoint p3)
{
    double x1 = p1.m_x; double x2 = p2.m_x; double x3 = p3.m_x;
    double y1 = p1.m_y; double y2 = p2.m_y; double y3 = p3.m_y;

    //calculate the angle difference
    double num1 = x2 - x1;
    double den1 = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
    double num2 = x3 - x2;
    double den2 = sqrt(pow(y3 - y2, 2) + pow(x3 - x2, 2));
    double angle1 = acos(num1 / den1); double angle2 = acos(num2 / den2);
    double angle = abs(angle1 - angle2);

    return angle;
}

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
    double numSegments();
    double getFData(int index1, int index2);
    double getGData(int index1, int index2);
    double getCpData(int index1, int index2);
    double getCxrData(int index1, int index2);

    bool init();

    void callback(nav_msgs::Odometry msg);
    void scanCallBack(sensor_msgs::LaserScan scan);

    void SetSpeed(double speed, double angle);
    void setPose(QList<double> to_set);
    void goToXYZ(geometry_msgs::Point goTo);
    void setCommand(QString cmd);
    void doMath(sensor_msgs::LaserScan scan);//does the matrix stuff
    void matchSegments();
    void finish();
    void run();
    Q_SIGNAL void NewPoint();

    globalPoint getPoint(int index);
    double getRosTime();
    // Mutex accessor methods
    void LockMutex(){m_Mutex.lock();}
    void UnlockMutex(){m_Mutex.unlock();}
    bool TryLockMutex(int timeOut = 0){if(timeOut) return m_Mutex.tryLock(); else return m_Mutex.tryLock(timeOut);}

private:
    Q_SIGNAL void rosShutdown();
    Q_SLOT void unlock();

    QString command;

    QMutex m_Mutex;

    int m_Init_argc;
    char** m_pInit_argv;

    int lockedIndex;

    bool m_locked;
    bool m_isDone;
    bool locked();

    double m_speed;
    double m_angle;

    double m_xPos;
    double m_yPos;
    double m_aPos;

    double m_maxRange;
    double m_minRange;

    void constructSegments();
    void breakPointList();
    void publishMap();

    QList<double> ranges;
    QList<double> map_cloud;

    std::vector<globalPoint, Eigen::aligned_allocator<globalPoint> > m_scanned;
    std::vector<segment, Eigen::aligned_allocator<segment> > m_Map;
    std::vector<segment, Eigen::aligned_allocator<segment> > m_segments;

    Matrix6x6d m_covariance;

    globalPoint m_lockedPoint;

    PointCloud m_map;

    ros::Time m_lastLock;
    ros::Time m_thisLock;

    ros::Publisher cmd_publisher;
    ros::Publisher sim_velocity;
    ros::Publisher map_publisher;

    ros::Subscriber pose_listener;
    ros::Subscriber scan_listener;
};
}//end namespace
#endif
