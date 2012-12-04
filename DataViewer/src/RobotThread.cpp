#include "RobotThread.h"

namespace data_server {
RobotThread::RobotThread(int argc, char** pArgv)
    :	m_Init_argc(argc),
        m_pInit_argv(pArgv),
        m_circleCount(0)
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
    ros::init(m_Init_argc, m_pInit_argv, "data_listener");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;

    data_listener = nh.subscribe("run_data", 1000, &RobotThread::dataCallback, this);
    start();
    return true;
}//set up the ros toys.

void RobotThread::dataCallback(std_msgs::String msg)
{
    /** This method reads the scan information as a string.

        Published:
            1. Number of Shapes
            2. Number of Points
            3. Number of Circles, Segments, Curves, Input
            4. Time of correction

        {Shapes %i, Points %i, Circles %i, Segments %i, Curves %i, Input %i}
    **/

    std::string str = msg.data;
    QString message = str.c_str();

    if (message.contains("Shapes"))
    {
        QString shapeCount = message;

        int i1 = 0;
        int i2 = 0;
        int i3 = 0;
        int i4 = 0;

        i2 = shapeCount.indexOf(" ") + 1;
        shapeCount.remove(i1, i2);
        i2 = shapeCount.size() - 1;
        i1 = shapeCount.indexOf(",");
        shapeCount.remove(i1, i2);

        if (shapeCount.contains(" "))
        {
            shapeCount.replace(" ", "");
        }//remove blank space.

        shapeCounts.push_back(shapeCount.toInt());
        averageShapes();
        Q_EMIT newShapeCount();

        i1 = message.indexOf(",");
        i3 = message.indexOf(",",i1 + 1 );
        i2 = message.indexOf("s");
        i2 = message.indexOf("s", i2 + 1);
        i1 = message.indexOf("s",i3 + 1 );

        i3 = i1 + 1;

        QString circleCount = message;
        circleCount.remove(0, i1 + 1);
        i2 = circleCount.indexOf(",");
        i3 += 1 + i2;
        circleCount.remove(i2, circleCount.size() - 1);
        circleCounts.push_back(circleCount.toInt());
        averageCircles();
        Q_EMIT newCircle();

        QString segmentCount = message;
        segmentCount.remove(0, i3);
        i1 = segmentCount.indexOf("s");
        segmentCount.remove(0, i1 + 1);
        i2 = segmentCount.indexOf(",");
        segmentCount.remove(i2, segmentCount.size() - 1);
        segmentCounts.push_back(segmentCount.toInt());
        averageSegments();
        Q_EMIT newSegment();
    }

}//callback method for the strings

double RobotThread::getAverage(QList<int> list)
{
    int sum = 0;

    for (int x = 0; x < list.size(); x++)
        sum += list.at(x);

    return ((double) sum ) / ((double) list.size());
}

void RobotThread::averageShapes()
{
    m_shapeCount = (int) getAverage(shapeCounts);
}//end void

void RobotThread::averageCircles()
{
    m_circleCount = (int) getAverage(circleCounts);
}//average the circle count

void RobotThread::averageSegments()
{
    m_segmentCount = (int) getAverage(segmentCounts);
}

/** For a real robot
void RobotThread::callback(nav_msgs::Odometry msg)
{
    m_xPos = msg.pose.pose.position.x;
    m_yPos = msg.pose.pose.position.y;
    m_aPos = msg.pose.pose.orientation.w;

    //ROS_INFO("Pose: (%f, %f, %f)", m_xPos, m_yPos, m_aPos);
    Q_EMIT newPose();
}//callback method to update the robot's position. **/

void RobotThread::callback(turtlesim::Pose msg)
{
    m_xPos = msg.x;
    m_yPos = msg.y;
    m_aPos = msg.theta;

    //Q_EMIT newPose();
}

void RobotThread::scanCallBack(sensor_msgs::LaserScan scan)
{
    m_maxRange = scan.range_max;
    m_minRange = scan.range_min;

    for (unsigned int x = 0; x < scan.ranges.size(); x++)
        ranges.push_back(ranges.at(x));
}//callback method for updating the laser scan data.

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
        turtlesim::Velocity cmd_msg;
        cmd_msg.angular = m_angle;
        cmd_msg.linear = m_speed;

        /** For Use with real robots:
        geometry_msgs::Twist cmd_msg;
        cmd_msg.linear.x = m_speed;
        cmd_msg.angular.z = m_angle;**/

        //cmd_publisher.publish(msg);
        //sim_velocity.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }//do ros things.
}

void RobotThread::SetSpeed(double speed, double angle)
{
    ROS_INFO("SetSpeed recieved");
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

const int & RobotThread::getCircleCount(){ return m_circleCount; }
const int & RobotThread::getSegmentCount(){ return m_segmentCount; }
const int & RobotThread::getBezierCount(){ return m_bezierCount; }
const int & RobotThread::getShapeCount(){ return m_shapeCount; }

double RobotThread::getXSpeed(){ return m_speed; }
double RobotThread::getASpeed(){ return m_angle; }

double RobotThread::getXPos(){ return m_xPos; }
double RobotThread::getYPos(){ return m_yPos; }
double RobotThread::getAPos(){ return m_aPos; }
}//end namespace

