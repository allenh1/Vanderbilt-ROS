#include "RobotThread.h"

namespace server {
RobotThread::RobotThread(int argc, char** pArgv)
    :	m_Init_argc(argc),
        m_pInit_argv(pArgv)
{ lockedIndex = 0; ROS_INFO("Robot thread constructed"); }

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
    map_publisher = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/map", 1000);
    pose_listener = nh.subscribe("/odom", 10, &RobotThread::callback, this);
    scan_listener = nh.subscribe("/scan", 1000, &RobotThread::scanCallBack, this);
    m_thisLock = ros::Time::now();
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

        m_segments.at(x).line.m_m = m;
        m_segments.at(x).line.m_q = q;
        m_segments.at(x).Rx = Rx; m_segments.at(x).Rxx = Rxx;
        m_segments.at(x).Ry = Ry; m_segments.at(x).Ryy = Ryy;
        m_segments.at(x).Rxy = Rxy; m_segments.at(x).N1 = N1;
        m_segments.at(x).N2 = N2; m_segments.at(x).T = T;
        m_segments.at(x).m = m; m_segments.at(x).q = q;
        m_segments.at(x).s = s; m_segments.at(x).t = t;

        if (N1 < N2)
        {
            //case: x = sy + t
            for (unsigned int y = 0; y < m_segments.at(x).m_points.size(); y++)
                m_segments.at(x).m_points.at(y).m_x = s * m_segments.at(x).m_points.at(y).m_y + t;
        }

        if (N1 >= N2)
        {
            //case: y = mx + q
            for (unsigned int y = 0; y < m_segments.at(x).m_points.size(); y++)
                m_segments.at(x).m_points.at(y).m_y = m * m_segments.at(x).m_points.at(y).m_x + q;
        }

        if (m_segments.at(x).m_length < 0.3)
        { m_segments.erase(m_segments.begin() + x); x--; }
    }//end for x

    matchSegments();
    publishMap();
}//construct the segments, apply correction.

void RobotThread::matchSegments()
{
    for (unsigned int x = 0; x < m_segments.size(); x++)
    {
        for (unsigned int y = 0; y < m_Map.size(); y++)
        {
            if (getLineError(m_Map.at(y), m_segments.at(x)) < getMaxError(m_segments.at(x)))
            {   m_segments.erase(m_segments.begin() + x); x--; break; }//end if
        }//end for y
    }//end for x

    for (unsigned int z = 0; z < m_segments.size(); z++)
    {
        m_Map.push_back(m_segments.at(z));
    }//update z map.
}//match segments to the ones on the map.

void RobotThread::publishMap()
{
    if (m_Map.size() > 0)
    {
        PointCloud currentMap = m_map;
        currentMap.header.frame_id = "/map";
        currentMap.header.stamp = ros::Time::now();

        for (unsigned int x = 0; x < m_Map.size(); x++)
        {
            for (unsigned int y = 0; y < m_Map.at(x).m_points.size(); y++)
            {
                pcl::PointXYZ point;
                point.x = m_Map.at(x).m_points.at(y).m_x;
                point.y = m_Map.at(x).m_points.at(y).m_y;
                point.z = 0;

                currentMap.points.push_back(point);
            }//end for y.
        }//end for x.

        m_map = currentMap;
        currentMap.height = 1;
        currentMap.width = currentMap.points.size();

        pcl::PCDWriter writer;
        writer.write<pcl::PointXYZ> ("Map.pcd", currentMap, false);

        if (m_Map.size() > 5000)
            m_Map.clear();//clear if the map is getting full
        m_segments.clear();
        m_scanned.clear();
        map_publisher.publish(currentMap.makeShared());
    }
}

double RobotThread::getRosTime()
{
    double time = ros::Time::now().toSec();
    return time;
}//get the time as a string

void RobotThread::doMath(sensor_msgs::LaserScan scan)
{
    double alpha = m_minRange;
    QList<double> alphas;

    for (double y = 0; y < scan.angle_max; y += scan.angle_increment)
        alphas.push_back(y);

     double rangeDev = 0.00031;//sick estimate
     double alphaDev = 0.00047;//for covariance

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
        double Cxr20 = 0; double Cxr21 = 0; double Cxr22 = 0;

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

        if (!TryLockMutex(1000))
        {
            m_lastLock = m_thisLock;
            m_lockedPoint = toPush;
            m_thisLock = ros::Time::now();
        }
        //unlock in the event that TryLockMutex returns true
        UnlockMutex();
        m_scanned.push_back(toPush);
        alpha += scan.angle_increment;
        Q_EMIT NewPoint();
    }//iterate for all scans

    constructSegments();
}//calculate matrices for each scan reading.

double RobotThread::getFData(int index1, int index2){ return m_lockedPoint.F(index1, index2); }
double RobotThread::getGData(int index1, int index2){ return m_lockedPoint.G(index1, index2); }
double RobotThread::getCpData(int index1, int index2){ return m_lockedPoint.Cp(index1, index2); }
double RobotThread::getCxrData(int index1, int index2){ return m_lockedPoint.Cxr(index1, index2); }
double RobotThread::numSegments(){ return m_Map.size(); }
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

void RobotThread::finish(){ m_isDone = true; }

void RobotThread::run()
{
    ros::Rate loop_rate(1);
    command = "empty";
    m_isDone = false;
    while (ros::ok() && !m_isDone)
    {
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
