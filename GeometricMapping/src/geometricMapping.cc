#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry.h"
#include "Shape.h"

void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    PointCloud original;

    iterator = 0;
    Shapes.clear();
    Segments.clear();
    Circles.clear();
    Angles.clear();
    projector_.projectLaser(*scan_in, cloud);

    for (unsigned int x = 0; x < cloud.points.size(); ++x)
    {
        double py = cloud.points.at(x).y;
        double px = cloud.points.at(x).x;
        double pZ = Z;

        pcl::PointXYZ toPush;
        toPush.x = px; toPush.y = py; toPush.z = pZ;

        original.points.push_back(toPush);
    }//end for x.

    original.height = 1;
    original.width = original.points.size();

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ>("Unfiltered.pcd", original, false);
    defineObjects();

    for (unsigned int x = 0; x < Shapes.size(); ++x)
    {
        Shape circle_test(Shapes.at(x));
        scans.push_back(circle_test);
    }//end for x.

    makeOneCloud();
    if (runNumber < 3)
        outputData(Slopes, AngleDiff);
    runNumber++;
}//Converts the laser scan into a point cloud

int main(int argc, char **argv)
{
    ros::init(argc , argv , "laser_ReadOut");

    ros::NodeHandle handler;

    ros::Subscriber laserReader = handler.subscribe("scan", 10000, scanCallBack);
    laserOutput = handler.advertise<pcl::PointCloud<pcl::PointXYZ> >("/cloud_pcl", 100);

    ros::spin();

    return 0;
}//end main.
