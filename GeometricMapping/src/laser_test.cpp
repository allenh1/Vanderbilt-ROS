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
ros::Publisher laserOutput;
ros::Publisher data_Output;
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

void publishRunData()
{
    /** This method publishes the scan information as a string.

        Published:
            1. Number of Shapes
            2. Number of Points
            3. Number of Circles, Segments, Curves, Input
            4. Time of correction

        {Shapes %i, Points %i, Circles %i, Segments %i, Curves %i, Input %i}
    **/
    QString rawMessage = "{Shapes ";
    QString numOShapes, numOPoints, numCircles, numSegments, numCurves, numRaw;
    int numShapes = scans.size();
    int numPoints = 0;
    numOShapes.setNum(numShapes);
    numOShapes += ", Points ";
    rawMessage += numOShapes;

    int circles = 0;
    int unchanged = 0;
    int segments = 0;
    int curves = 0;

    for (unsigned int x = 0; x < scans.size(); x++)
    {
        numPoints += scans.at(x).getCorrections().points.size();

        int type = scans.at(x).getType(); //get the shape
        if (type == CIRCLE)
            circles++;
        else if (type == SEGMENT)
            segments++;
        else if (type == BEZIER)
            curves++;
        else
            unchanged++;
    }

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
    numRaw += "}";
    rawMessage += numRaw;

    std_msgs::String str;
    str.data = rawMessage.toStdString();

    data_Output.publish(str);
}//publish data as a string.

void defineLists()
{
    for (unsigned int x = 1; x < cloud.points.size() - 1; x++)
    {
        double num = cloud.points.at(x).y - cloud.points.at(x - 1).y;
        double den = cloud.points.at(x).x - cloud.points.at(x - 1).x;
        double slope = num / den;
        Slopes.push_back(slope);//push back the current slope

        pcl::PointXYZ projection;

        double x2 = cloud.points.at(x + 1).x; double x1 = cloud.points.at(x).x;
        double y2 = cloud.points.at(x + 1).y; double y1 = cloud.points.at(x).y;

        projection.x = x2;
        projection.y = slope * x2 + (y1 - slope * x1);
        projection.z = 0;
        projections.push_back(projection);//store the predicted next value

        //calculate the angle difference
        double num1 = x2 - x1;
        double den1 = sqrt(pow(projection.y - y1, 2) + pow(projection.x - x1, 2));
        double num2 = num1;
        double den2 = sqrt(pow(y2 - y1, 2) + pow(x2 - x1, 2));
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

    for (int x = 0; x < Angles.size() - 1; x++)
    {
        if ((Angles.at(x) >= PHIe && x - lastBreak > 2) ||
            (x - lastBreak > 2 && getDistance(toPCLPoint(cloud.points.at(x)), toPCLPoint(cloud.points.at(x + 1))) > DIST_TOLERANCE))
        {

            PointCloud toPush;

            for (int i = lastBreak; i + 1 <  x; i += 2 )
            {
                //ROS_INFO("In Assignment Loop: i = %i, x = %i, size = %i", i, x, cloud.points.size() - 1);
                pcl::PointXYZRGB p1;
                p1.x = cloud.points.at(i - 1).x; p1.y = cloud.points.at(i - 1).y;
                p1.z = 0;


                pcl::PointXYZRGB p2;
                p2.x = cloud.points.at(i).x; p2.y = cloud.points.at(i).y;
                p2.z = 0;

                pcl::PointXYZRGB p3;
                p3.x = cloud.points.at(i + 1).x; p3.y = cloud.points.at(i + 1).y;
                p3.z = 0;

                toPush.points.push_back(p1);
                toPush.points.push_back(p2);
                toPush.points.push_back(p3);
            }//end for i

            Shapes.push_back(toPush);
            lastBreak = x;
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
            pcl::PointXYZRGB toPush = currentCorrected.points.at(y);
            toPush.x = -py; toPush.y = pZ; toPush.z = px;

            combined.points.push_back(toPush);
            scores.push_back(scans.at(x).bestMatch());

            //print the shape
        }//end for y
    }//end for x.

    correctedCloud = combined;
    combined.height = 1;
    combined.width = combined.points.size();

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB> ("FinalCorrections.pcd", combined, false);
    combined.header.frame_id = "/laser";
    laserOutput.publish(combined.makeShared());

    publishRunData();
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

        pcl::PointXYZRGB toPush;
        toPush.x = px; toPush.y = py; toPush.z = pZ;

        original.points.push_back(toPush);
    }//end for x.

    original.height = 1;
    original.width = original.points.size();

    pcl::PCDWriter writer;
    writer.write<pcl::PointXYZRGB>("Unfiltered.pcd", original, false);
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

    ros::Subscriber laserReader = handler.subscribe("/scan", 10000, scanCallBack);
    laserOutput = handler.advertise<pcl::PointCloud<pcl::PointXYZ> >("/cloud_pcl", 100);
    data_Output = handler.advertise<std_msgs::String>("/run_data", 100);

    ros::spin();

    return 0;
}//end main.
