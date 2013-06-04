#include "RobotThread.h"

namespace data_server {
RobotThread::RobotThread(int argc, char** pArgv)
    :	m_Init_argc(argc),
      m_pInit_argv(pArgv)
{

}

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
    ROS_INFO("Init");
    ros::init(m_Init_argc, m_pInit_argv, "variance_tracker");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;
    m_isFirstMessage = true;
    nh.subscribe("/pose", 10, &RobotThread::callback, this);
    scan_listener = nh.subscribe("/scan", 1000, &RobotThread::scanCallBack, this);

    cloud_pcl = nh.advertise<pcl::PointCloud<pcl::PointXYZ> >("/cloud_pcl", 100);
    data_Output = nh.advertise<std_msgs::String>("/run_data", 100);

    start();

    return true;
}//set up the ros toys.

void RobotThread::callback(nav_msgs::Odometry msg)
{
    m_xPos = msg.pose.pose.position.x;
    m_yPos = msg.pose.pose.position.y;
    m_aPos = msg.pose.pose.orientation.w;

    //ROS_INFO("Pose: (%f, %f, %f)", m_xPos, m_yPos, m_aPos);
}//callback method to update the robot's position.

inline double getMean(QList<double> list)
{
    int N = list.size();
    double sum = 0;

    for (int x = 0; x < list.size(); x++)
        sum += list.at(x);

    return sum / (double) N;
}//returns the arithmetic mean

inline double getVariance(QList<double> _list)
{
    QList<double> list = _list;

    double mu = getMean(list);
    double N = list.size();
    double sum = 0;

    for (int x = 0; x < list.size(); x++)
     {
        double temp = list.at(x);
        sum += (temp - mu) * (temp - mu);

    }
    return sum / N;
}//returns the variance of the data

void RobotThread::scanCallBack(sensor_msgs::LaserScan scan)
{
    /** This method is used to measure the variance for each beam. **/

    double theta = scan.angle_min;
    QList<LaserPoint> scan_data;
    double bearing = theta;
    for (unsigned int x = 0; x < scan.ranges.size(); x++)
    {
        /** This projects the laser scan **/
        double range = scan.ranges.at(x);

        if (scan.ranges.at(x) == scan.range_max)
        { if (x == scan.ranges.size() - 2){ range = 0; } else { while(x < scan.ranges.size() && scan.ranges.at(x) == scan.range_max){range = scan.ranges.at(x); x++;} }}


        double px = m_xPos + range * cos(theta + bearing + PI / 2);
        double py = m_yPos + range * sin(theta + bearing + PI / 2);

        LaserPoint toPush;
        toPush.bearing = bearing;
        toPush.range = range;
        toPush.px = px;
        toPush.py = py;

        bearing += scan.angle_increment;
        scan_data.push_back(toPush);
    }//end for x

    std_msgs::Header toPass = scan.header;

    extractCurves(scan_data, toPass);
}//callback method for updating the laser scan data.

inline double getDistance(LaserPoint a, LaserPoint b)
{ return sqrt(pow(a.px - b.px, 2) + pow(a.py - b.py, 2)); }

void RobotThread::publishRunData(QList<Shape> scan)
{
    QList<Shape> scans = scan;
    /** This method publishes the scan information as a string.

        Published:
            1. Number of Shapes
            2. Number of Points
            3. Number of Circles, Segments, Curves, Input
            4. Max Dist Shift for Shape
            5. Time of correction

        {Shapes %i, Points %i, Circles %i, Segments %i, Curves %i, Input %i, MaxSeg %i, @T: time}
    **/
    QString rawMessage = "{Shapes ";
    QString numOShapes, numOPoints, numCircles, numSegments, numCurves, numRaw, segRaw, timeRaw;
    int numShapes = scans.size();
    int numPoints = 0;
    numOShapes.setNum(numShapes);
    numOShapes += ", Points ";
    rawMessage += numOShapes;

    int circles = 0;
    int unchanged = 0;
    int segments = 0;
    int curves = 0;
    double maxDist = 0;

    for (int x = 0; x < scans.size(); x++)
    {
        Shape current = scans.at(x);
        numPoints += current.getCorrections().points.size();

        int type = current.getType(); //get the shape
        if (type == CIRCLE)
            circles++;
        else if (type == SEGMENT)
            segments++;
        else if (type == BEZIER)
            curves++;
        else
            unchanged++;
    }

    for (int x = 0; x < scans.size(); x++)
    {
        Shape current = scans.at(x);

        int type = current.getType();

        if (type == SEGMENT)
        {
            if (maxDist < scans.at(x).max_correct)
                maxDist = scans.at(x).max_correct;
        }//end if
    }//iterate through to get the max correction distance.

    numOPoints.setNum(numPoints);
    numOPoints += ", Circles ";
    rawMessage += numOPoints;

    numCircles.setNum(circles);
    numCircles +=", Segments ";
    rawMessage += numCircles;

    numSegments.setNum(segments);
    numSegments += ", Curves ";
    rawMessage += numSegments;

    numCurves.setNum(curves);
    numCurves += ", Input ";
    rawMessage += numCurves;

    numRaw.setNum(unchanged);
    numRaw += ", MaxSegError ";
    rawMessage += numRaw;

    segRaw.setNum(maxDist);
    segRaw += "} @T: ";
    rawMessage += segRaw;

    timeRaw.setNum(ros::Time::now().toNSec());
    rawMessage += timeRaw;

    std_msgs::String str;
    str.data = rawMessage.toStdString();

    data_Output.publish(str);
}//publish data as a string.

void RobotThread::extractCurves(QList<LaserPoint> scan_data, std_msgs::Header scanTime)
{
    /** This is the feature extraction method:
     *
     * 1. Calculate the maximum length of a laser scan along both sides of a range reading i: K_f[i], K_b[i].
     *    K_f[i] is the max range that satisfies d(i, i + K_f[i]) > l(i, i + K_f[i]) - 1
     * 2. Get angle
     * 3. Construct features, composed of at least 10 range readings. **/

    QList<Shape> shapes;

    PointCloud current;
    bool addTo = true;

    for (int x = 2; x < scan_data.size(); x++)
    {
        if (!addTo)
        { shapes.push_back(Shape(current)); current.clear(); addTo = true; }//start over

        //We start at one so that we can access the previous
        int iter = x + 1;
        int k_f = x + 1;
        int k_b = x - 1;

        while (iter < scan_data.size())
        {
            /** Determine K_f **/
            double dist = getDistance(scan_data.at(x), scan_data.at(iter));
            double l = scan_data.at((x + iter) / 2).range - 1;

            if (iter >= scan_data.size() - 1 || dist < l)
            { k_f = iter; break; }

            iter++;
        }//end while

        iter = x - 1;

        while (iter > 0)
        {
            double dist = getDistance(scan_data.at(x), scan_data.at(iter));
            double l = scan_data.at((x + iter) / 2).range - 1;

            if (iter == 0 || dist < l)
            { k_b = iter; break; }
            iter--;
        }//end while

        if (x < scan_data.size() - 1)
        {
            double f_ix = scan_data.at(k_f).px - scan_data.at(x).px;
            double f_iy = scan_data.at(k_f).py - scan_data.at(x).py;

            double b_ix = scan_data.at(k_b).px - scan_data.at(x).py;
            double b_iy = scan_data.at(k_b).py - scan_data.at(x).py;

            double f_iDotb_i = f_ix * b_ix + f_iy * b_iy;
            double f_iTimesb_i = sqrt(pow(f_ix, 2) + pow(f_iy, 2)) * sqrt(pow(b_ix, 2) + pow(b_iy, 2));

            double theta_i = acos(f_iDotb_i / f_iTimesb_i);

            if (sqrt(pow(theta_i, 2)) > 0.1 && current.size() >= 10)
                addTo = false;

            else
            {
                pcl::PointXYZ toPush;
                toPush.x = scan_data.at(x).px;
                toPush.y = scan_data.at(x).py;
                toPush.z = 1.0; //set a standard height.

                current.push_back(toPush);
            }//end else
        }//end for
    }

    PointCloud final;

    publishRunData(shapes);

    for (int y = 0; y < shapes.size(); y++)
    {
        Shape current = shapes.at(y);
        PointCloud curRent = current.getCorrections();

        for (unsigned int z = 0; z < curRent.size(); z++)
        {

            pcl::PointXYZ toPush;
            toPush.x = -curRent.points.at(z).y;
            toPush.y = curRent.points.at(z).z;
            toPush.z = curRent.points.at(z).x;

            final.push_back(toPush);
        }//END FOR
    }
    final.height = 1;
    final.width = final.points.size();
    final.header.frame_id = "/laser";
    final.header.stamp = scanTime.stamp;

    cloud_pcl.publish(final.makeShared());
}//end callback

void RobotThread::run()
{

    ros::Rate loop_rate(1);
      while (ros::ok())
      {
          ros::spinOnce();
          loop_rate.sleep();
      }//do ros things.
}

}//end namespace

