#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry.h"
#include "Shape.h"
#include <qt4/QtCore/qconfig.h>
#include <qt4/Qt/qglobal.h>
#include <qt4/QtCore/qnamespace.h>
#include <qt4/QtCore/qobjectdefs.h>
#include <qt4/Qt/qobject.h>
#include <qt4/QtCore/qcoreapplication.h>
#include <qt4/QtCore/QObject>
#include <qt4/QtCore/qobject.h>
#include <qt4/QtGui/QApplication>
#include <qt4/QtCore/QtConfig>
#include <qt4/QtCore/qiterator.h>
#include <qt4/QtCore/QtGlobal>
#include <qt4/QtCore/QList>

double prevPhiE = PI/2;
int matchIndex = 0;
int runNumber = 1;
ros::Publisher  laserOutput;
laser_geometry::LaserProjection projector_;
sensor_msgs::PointCloud cloud;
sensor_msgs::PointCloud toPublish;
PointCloud correctedCloud;
sensor_msgs::LaserScan converted;
std::vector<PointCloud, Eigen::aligned_allocator<PointCloud> > Shapes;
std::vector<Shape, Eigen::aligned_allocator<Shape> > scans;
std::vector<Shape, Eigen::aligned_allocator<Shape> > Segments;
std::vector<Shape, Eigen::aligned_allocator<Shape> > Circles;
QList<indexSegment> slopeVector;
QList<double> Slopes;
QList<double> Angles;
QList<double> AnglesStored;
QList<pcl::PointXYZ> projections;
QList<double> scores;

inline double abs(double num)
{ return (num < 0) ? num * -1 : num; }//returns the absolute value of a double, num.

void defineLists()
{
    for (unsigned int x = 1; x < cloud.points.size() - 1; x++)
    {
        double num = cloud.points.at(x).y - cloud.points.at(x - 1).y;
        double den = cloud.points.at(x).x - cloud.points.at(x - 1).x;
        double slope = num / den;
        Slopes.push_back(slope);//push back the current slope

        pcl::PointXYZ projection;

        projection.x = cloud.points.at(x + 1).x;
        projection.y = slope * cloud.points.at(x + 1).x + (cloud.points.at(x).y - slope * cloud.points.at(x).x);
        projection.z = 0;
        projections.push_back(projection);//store the predicted next value

        //calculate the angle difference
        double num1 = cloud.points.at(x + 1).x - cloud.points.at(x).x;
        double den1 = sqrt(pow(projection.y - cloud.points.at(x).y, 2) + pow(projection.x - cloud.points.at(x).x, 2));
        double num2 = num1;
        double den2 = sqrt(pow(cloud.points.at(x + 1).y - cloud.points.at(x).y, 2) + pow(cloud.points.at(x + 1).x - cloud.points.at(x).x, 2));

        double angle1 = acos(num1 / den1); double angle2 = acos(num2 / den2);
        double angle = abs(angle1 - angle2);

        Angles.push_back(angle);//store the angle between the actual and the predicted y values.
        AnglesStored.push_back((angle * 180) / PI);
    }//end for x.
}//define the slopes and phi measurements.

void defineObjects()
{
    defineLists();
    int lastBreak = 1;
    //ROS_INFO("In defineObjects()");
    //ROS_INFO("Angles.size() = %i", Angles.size());

    for (int x = 0; x < Angles.size(); x++)
    {
        if (Angles.at(x) >= PHIe && x - lastBreak > 2)
        {
            //ROS_INFO("Angle violated.");

            PointCloud toPush;

            for (int i = lastBreak; i <  x; i += 2)
            {
                //ROS_INFO("In Assignment Loop: i = %i, x = %i, size = %i", i, x, cloud.points.size() - 1);
                pcl::PointXYZ p1;
                p1.x = cloud.points.at(i - 1).x; p1.y = cloud.points.at(i - 1).y;
                p1.z = 0;


                pcl::PointXYZ p2;
                p2.x = cloud.points.at(i).x; p2.y = cloud.points.at(i).y;
                p2.z = 0;

                pcl::PointXYZ p3;
                p3.x = cloud.points.at(i + 1).x; p3.y = cloud.points.at(i + 1).y;
                p3.z = 0;

                toPush.points.push_back(p1);
                toPush.points.push_back(p2);
                toPush.points.push_back(p3);
            }//end for i

            Shapes.push_back(toPush);
            lastBreak = x + 2;
        }//add to cloud
    }//end for
}//get a single object

void sortClouds()
{
    for (unsigned int x = 0; x < scans.size(); ++x)
    {
        if (scans.at(x).is_segment())
            Segments.push_back(scans.at(x));
        else
            Circles.push_back(scans.at(x));
    }//end for x.
}//sorts the clouds into semantic shape tags.

bool hasMatch(Shape segment)
{
    for (int x = 0; x < slopeVector.size(); ++x)
    {
        if (abs(slopeVector.at(x).slope - segment.getSlope()) <= SLOPE_TOLERATION)
        { matchIndex = x; return true; }
    }//end for

    matchIndex = -1;
    return false;
}//check for a possible matching slope.

void matchSlopes()
{
    //for all the segments, sort them into a similar group. Create a tag for each.
    //They have a data structure that cooresponds to the indexSlope structure.

    for (unsigned int x = 0; x < Segments.size(); ++x)
    {
        if (hasMatch(Segments.at(x)) && matchIndex != -1)
        {
            double currentSlope = slopeVector.at(matchIndex).slope;

            Segments.at(x).setSlope(currentSlope);
        }//end if.
    }//end for x.
}//combine close segments into a line.

void makeOneCloud()
{
    PointCloud combined;
    //matchSlopes();

    for (unsigned int x = 0; x < scans.size(); ++x)
    {
        PointCloud currentCorrected = scans.at(x).getCorrections();
        for (unsigned int y = 0; y < currentCorrected.points.size(); ++y)
        {
            double py = currentCorrected.points.at(y).y;
            double px = currentCorrected.points.at(y).x;
            double pZ = Z;
            //calculate the projeciton
            pcl::PointXYZ toPush;
            toPush.x = -py; toPush.y = pZ; toPush.z = px;

            combined.points.push_back(toPush);
            scores.push_back(scans.at(x).bestMatch());

        }//end for y
    }//end for x.

    correctedCloud = combined;
    combined.height = 1;
    combined.width = combined.points.size();

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZ> ("FinalCorrections.pcd", combined, false);
    combined.header.frame_id = "/laser";
    laserOutput.publish(combined.makeShared());

    scans.clear();
}//this method combines all the corrected shapes to a single point cloud.

void scanCallBack(const sensor_msgs::LaserScan::ConstPtr& scan_in)
{
    PointCloud original;

    Shapes.clear();
    scores.clear();
    Segments.clear();
    Circles.clear();
    AnglesStored.clear();
    Slopes.clear();
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
        outputData(scores, AnglesStored);
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
