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
    ros::init(m_Init_argc, m_pInit_argv, "variance_tracker");

    if (!ros::master::check())
        return false;//do not start without ros.

    ros::start();
    ros::Time::init();
    ros::NodeHandle nh;
    m_isFirstMessage = true;
    scan_listener = nh.subscribe("scan", 1000, &RobotThread::scanCallBack, this);
    start();

    return true;
}//set up the ros toys.

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

    QList<double> ranges;

    for (unsigned int x = 0; x < scan.ranges.size(); x++)
        ranges.push_back(scan.ranges.at(x));

    if (m_isFirstMessage)
    {
        for (int x = 0; x < ranges.size(); x++)
        {
            LaserData current; current.m_ranges.push_back(ranges.at(x));
            current.m_mean = 1.0; current.m_variance = 1.0;
            m_pastData.push_back(current);
        }//end for x.

        m_isFirstMessage = false;
    }

    else
    {
        for (int x = 0; x < m_pastData.size(); x++)
        {
            LaserData current = m_pastData.at(x);
            current.m_ranges.push_back(ranges.at(x));
            current.m_mean = getMean(m_pastData.at(x).m_ranges);
            current.m_variance = getVariance(m_pastData.at(x).m_ranges);

            //std::cout<<"Variance: " << current.m_variance <<" \n";
            m_pastData.replace(x, current);
        }//end for x
    }
}//callback method for updating the laser scan data.

void RobotThread::run()
{    ros::Rate loop_rate(1);
      while (ros::ok())
      {
          ros::spinOnce();
          loop_rate.sleep();
      }//do ros things.
}

void RobotThread::saveData()
{
    /** Export the data to a csv type file **/
    ROS_INFO("saving data");

    ROS_INFO("data size: %i", m_pastData.size());

    QFile file("Data.txt");

    if (!file.open(QIODevice::WriteOnly))
        return;

    QList<LaserData> current = m_pastData;
    QString output = "";

    for (int x = 0; x < current.size(); x++)
    {
        QString num1;
        num1.setNum(current.at(x).m_mean);
        QString num2;
        num2.setNum(current.at(x).m_variance, 'g', 10);

        output += num1 + "\t";
        output += num2;
        output += "\n";
    }//add data to the outuput

    QTextStream stream(&file);

    stream<<output;
    stream.flush();
    file.close();
}//write the data to a file

}//end namespace

