#include "RobotThread.h"

namespace server {
RobotThread::RobotThread(int argc, char** pArgv)
    :	m_Init_argc(argc),
        m_pInit_argv(pArgv)
{ lockedIndex = 0; m_locked = false; ROS_INFO("Robot thread constructed"); }

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
    ros::init(m_Init_argc, m_pInit_argv, "RobotThread");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;
    //rostopic pub p2os_driver/MotorState cmd_motor_state -- 1.0
    cmd_publisher = nh.advertise<std_msgs::String>("/tcp_cmd", 1000);
    map_publisher = nh.advertise<nav_msgs::OccupancyGrid>("/map", 1000);
    pose_listener = nh.subscribe("/odom", 10, &RobotThread::callback, this);
    scan_listener = nh.subscribe("/scan", 1000, &RobotThread::scanCallBack, this);
    start();
    return true;
}//set up the ros toys.

void RobotThread::callback(nav_msgs::Odometry msg)
{
    m_xPos = msg.pose.pose.position.x;
    m_yPos = msg.pose.pose.position.y;
    m_aPos = msg.pose.pose.orientation.w;

    int row = 0;

    for (unsigned int x = 0; x < msg.pose.covariance.size(); x++)
    {
        if (x > 0 && x % 6 == 0)
            row++;

        m_covariance(row, x % 5) = msg.pose.covariance.at(x);
    }//end for x (copy the covariance to a matrix
}//callback method to update the robot's position.

void RobotThread::scanCallBack(sensor_msgs::LaserScan scan)
{
    ranges.clear();
    m_maxRange = scan.range_max;
    m_minRange = scan.range_min;

    for (unsigned int x = 0; x < scan.ranges.size(); x++)
        ranges.push_back(scan.ranges.at(x));

    doMath(scan);
}//callback method for updating the laser scan data.

void RobotThread::breakPointList()
{
    std::vector<globalPoint, Eigen::aligned_allocator<globalPoint> > currentSet;

    for (unsigned int x = 1; x < m_scanned.size() - 1; x++)
    {
        bool canAdd = true;

        if (!(getDistance(m_scanned.at(x), m_scanned.at(x - 1)) > dcritl))
            canAdd = false;

        if (!(getAngleDif(m_scanned.at(x - 1), m_scanned.at(x), m_scanned.at(x + 1)) < alpha_crit))
            canAdd = false;

        if (canAdd)
            currentSet.push_back(m_scanned.at(x));

        if (currentSet.size() > 1 && !canAdd)
        {
            segment toPush;
            toPush.m_length = getDistance(currentSet.at(0), currentSet.at(currentSet.size() - 1));

            for (unsigned int y = 0; y < currentSet.size(); y++)
                toPush.m_points.push_back(currentSet.at(y));

            m_segments.push_back(toPush);
            currentSet.clear();
        }//end if
    }//end for x
}//break the points into segments

void RobotThread::constructSegments()
{
    breakPointList();
    //This method constructs the segments through the collection of points.

    for (unsigned int x = 0; x < m_segments.size(); x++)
    {
        double Rx = 0; double Ry = 0; double Rxx = 0;
        double Ryy = 0; double Rxy = 0; double N1 = 0; double N2 = 0; double T = 0;
        double m = 0; double q = 0; double s = 0; double t = 0;

        QList<double> xVals; QList<double> yVals;

        for (unsigned int y = 0; y < m_segments.at(x).m_points.size(); y++)
        {
            xVals.push_back(m_segments.at(x).m_points.at(y).m_x);
            yVals.push_back(m_segments.at(x).m_points.at(y).m_y);
        }//end for y.

        Rx  = summation(xVals);                     Ry = summation(yVals);
        Rxx = summation(raiseListToPow(xVals, 2)); Ryy = summation(raiseListToPow(yVals, 2));

        Rxy = summation(multiplyIndecies(xVals, yVals));

        N1 = Rxx * m_segments.at(x).m_points.size() - pow(Rx, 2);
        N2 = Ryy * m_segments.at(x).m_points.size() - pow(Ry, 2);

        T  = Rxy * m_segments.at(x).m_points.size() - Rx * Ry;

        m = T / N1;

        q = (Ry - m * Rx) / m_segments.at(x).m_points.size();

        s = T / N2;

        t = (Rx - s * Ry) / m_segments.at(x).m_points.size();

        if (N1 <= N2)
        {
            for (unsigned int z = 0; z < m_segments.at(x).m_points.size(); z++)
            {
                globalPoint p = m_segments.at(x).m_points.at(z);
                p.m_x = s * p.m_y + t;

                m_Map.push_back(p);
            }//end for z
        }//case: x = sy + t

        else
        {
            for (unsigned int z = 0; z < m_segments.at(x).m_points.size(); z++)
            {
                globalPoint p = m_segments.at(x).m_points.at(z);
                p.m_y = m * p.m_x + q;

                m_Map.push_back(p);
            }//end for z.
        }//case: y = mx + q
    }//end for x
}//construct the segments, apply correction.

void RobotThread::publishMap()
{
    nav_msgs::OccupancyGrid myMap;

    //Message Header
    myMap.header.stamp = ros::Time::now();
    myMap.header.frame_id = "/map";

    //Map Meta Data Info
    myMap.info.resolution = 0.050000; //standard resolution
    myMap.info.width = 100;
    myMap.info.height = 100;

    geometry_msgs::Pose origin;
    origin.position.x = -100;
    origin.position.y = -100;
    origin.position.z = 0;
}

void RobotThread::doMath(sensor_msgs::LaserScan scan)
{
    double alpha = m_minRange;
    QList<double> alphas;

    for (double y = 0; y < scan.angle_max; y += scan.angle_increment)
        alphas.push_back(y);

     double rangeDev = stdDev(ranges);
     double alphaDev = stdDev(alphas);

    for (unsigned int x = 0; x < scan.ranges.size(); x++)
    {
        globalPoint toPush;
        toPush.m_x = m_xPos + Delta_x * cos(m_aPos) - Delta_y * sin(m_aPos) + scan.ranges.at(x) * cos(alpha + m_aPos);
        toPush.m_y = m_yPos + Delta_x * sin(m_aPos) + Delta_y * cos(m_aPos) + scan.ranges.at(x) * sin(alpha + m_aPos);

        double F20 = - Delta_x * sin(m_aPos) - Delta_y * cos(m_aPos) - scan.ranges.at(x) * sin(alpha + m_aPos);
        double F21 =   Delta_x * cos(m_aPos) - Delta_y * sin(m_aPos) + scan.ranges.at(x) * cos(alpha + m_aPos);

        toPush.F(0, 0) = 1; toPush.F(0, 1) = 0; toPush.F(0, 2) = F20;
        toPush.F(1, 0) = 0; toPush.F(1, 1) = 1; toPush.F(1, 2) = F21;

        double G00 = cos(alpha + m_aPos); double G01 = - scan.ranges.at(x) * sin(alpha + m_aPos);
        double G10 = sin(alpha + m_aPos); double G11 =   scan.ranges.at(x) * cos(alpha + m_aPos);

        toPush.G(0, 0) = G00; toPush.G(0, 1) = G01;
        toPush.G(1, 0) = G10; toPush.G(1, 1) = G11;

        double Cs00 = rangeDev; double Cs01 = 0;
        double Cs10 = 0;        double Cs11 = alphaDev;

        toPush.Cs(0, 0) = Cs00; toPush.Cs(0, 1) = Cs01;
        toPush.Cs(1, 0) = Cs10; toPush.Cs(1, 1) = Cs11;

        double Cxr00 = m_covariance(0, 0); double Cxr01 = m_covariance(0, 1); double Cxr02 = m_covariance(0, 2);
        double Cxr10 = m_covariance(1, 0); double Cxr11 = m_covariance(1, 1); double Cxr12 = m_covariance(1, 2);
        double Cxr20 = m_covariance(3, 0); double Cxr21 = m_covariance(3, 1); double Cxr22 = m_covariance(3, 2);

        toPush.Cxr(0, 0) = Cxr00; toPush.Cxr(0, 1) = Cxr01; toPush.Cxr(0, 1) = Cxr02;
        toPush.Cxr(1, 0) = Cxr10; toPush.Cxr(1, 1) = Cxr11; toPush.Cxr(1, 2) = Cxr12;
        toPush.Cxr(2, 0) = Cxr20; toPush.Cxr(2, 1) = Cxr21; toPush.Cxr(2, 2) = Cxr22;

        Matrix2x3d mult1; Matrix2x2d mult2;
        mult1 = toPush.F * toPush.Cxr;
        mult2 = mult1 * toPush.F.transpose();

        Matrix2x2d mult3; Matrix2x2d mult4;
        mult3 = toPush.G * toPush.Cs;
        mult4 = mult3 * toPush.G.transpose();

        toPush.Cp = mult2 + mult4; // store the uncertainty here.

        if (!TryLockMutex(1000)){
            m_lockedPoint = toPush;
        }
        //unlock in the event that TryLockMutex returns true
        UnlockMutex();
        m_scanned.push_back(toPush);
        alpha += scan.angle_increment;
        Q_EMIT NewPoint();
    }//iterate for all scans

    m_Map.clear();
    constructSegments();
}//calculate matrices for each scan reading.

double RobotThread::getFData(int index1, int index2){ return m_lockedPoint.F(index1, index2); }
double RobotThread::getGData(int index1, int index2){ return m_lockedPoint.G(index1, index2); }
double RobotThread::getCpData(int index1, int index2){ return m_lockedPoint.Cp(index1, index2); }
double RobotThread::getCxrData(int index1, int index2){ return m_lockedPoint.Cxr(index1, index2); }
globalPoint RobotThread::getPoint(int index) { return m_scanned.at(index); }//returns the newest point to the gui
bool RobotThread::locked(){ return m_locked; }
void RobotThread::unlock(){ m_locked = false; }
//void RobotThread::lock(){ m_locked = true; }

void RobotThread::goToXYZ(geometry_msgs::Point goTo)
{
    MoveBaseClient ac("move_base", true);

    while(!ac.waitForServer(ros::Duration(5.0))){
      ROS_INFO("Waiting for the move_base action server to come up");
    }

    move_base_msgs::MoveBaseGoal goal;

    //we'll send a goal to the robot to move 1 meter forward
    goal.target_pose.header.frame_id = "base_link";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = goTo.x;
    goal.target_pose.pose.position.y = goTo.y;
    goal.target_pose.pose.orientation.w = goTo.z; // the angle will be on z.

    ROS_INFO("Sending goal");
    ac.sendGoal(goal);

    ac.waitForResult();

    if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
      ROS_INFO("Hooray, the base moved 1 meter forward");
    else
      ROS_INFO("The base failed to move forward 1 meter for some reason");
}//go to pose

void RobotThread::run()
{
    ros::Rate loop_rate(1);
    command = "empty";
    while (ros::ok())
    {
        if (m_Map.size() > 1000)
            m_Map.clear();
        std_msgs::String msg;
        std::stringstream ss;
        ss << command.toStdString();

        msg.data = ss.str();

        geometry_msgs::Twist cmd_msg;
        cmd_msg.linear.x = m_speed;
        cmd_msg.angular.z = m_angle;

        //cmd_publisher.publish(msg);
        //sim_velocity.publish(cmd_msg);
        ros::spinOnce();
        loop_rate.sleep();
    }//do ros things.
    Q_EMIT rosShutdown();
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
