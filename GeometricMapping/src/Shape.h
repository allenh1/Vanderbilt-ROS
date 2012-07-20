#ifndef ROBOT_H
#define ROBOT_H
#include<iostream>
#include<pcl_ros/point_cloud.h>
#include<pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include<Eigen/StdVector>
#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "laser_geometry.h"
#include "MathLibrary.h"

const float PHIe =  PI / 8.0; // ~ 18 degrees
const double SLOPE_TOLERATION = 0.5; // difference is less than 0.25
const double DIST_TOLERANCE = 0.5; //70 cm distance tolerance.
const double DIST_TOL = 0.3;
const double MAXRAD = 0.75; //maximum radius of 2.0 meters
const double R2TOLL = 0.9995;
const double R2TOLL2 = 1.01;
typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

struct indexSegment
{
    double slope;
    unsigned int index;
};

const float min_dist = 0.7; //50 cm
int asdf = 1;

class Shape
{
public:
    Shape(PointCloud object); // constructor
    PointCloud getCorrections();
    void setSlope(double x);

    double bestMatch();
    double getSlope();
    double firstX;
    double finalX;
    double phiBound;

    bool is_circle();
    bool is_segment();

private:
    PointCloud uncorrected;
    PointCloud correctedSegment;
    PointCloud correctedCircle;

    void correct_circle();
    void correct_segment();
    void updateSegment();

    double match_circle();
    double match_segment();
    double meanY();
    double getAverageSlope();

    double S_m; //slope of correction.
    double S_b; //intercept of correction.
    double C_k; //the k parameter of a circle.
    double C_h; //the h parameter of a circle.
    double C_r; //the radius of the circle
};

#endif

pcl::PointXYZ toPCLPoint(geometry_msgs::Point32 in)
{
    pcl::PointXYZ out;
    out.x = in.x;
    out.y = in.y;
    out.z = Z;

    return out;
}//converts the input geometry messages point to a PCL point.

Shape::Shape(PointCloud object)
{
    uncorrected = object;
    const int last = uncorrected.points.size() - 1;
    firstX = uncorrected.points.at(0).x;
    finalX = uncorrected.points.at(uncorrected.points.size() - 1).x;

    phiBound = acos(sqrt(pow(uncorrected.points.at(0).x, 2) + pow(uncorrected.points.at(0).y, 2)) /
                    sqrt(pow(uncorrected.points.at(last).x, 2) + pow(uncorrected.points.at(last).y, 2)));

    correct_circle();
    correct_segment();
}

double Shape::bestMatch()
{     
    if (match_circle() > match_segment())
        return match_circle();
    else
        return match_segment();
}//end double

double Shape::getAverageSlope()
{
    QList<double> slopes;

    if (uncorrected.points.size() == 1)
        return 0;
    for (unsigned int x = 0; x < uncorrected.points.size() - 1; x++)
    {
        double ydiff = uncorrected.points.at(x + 1).y - uncorrected.points.at(x).y;
        double xdiff = uncorrected.points.at(x + 1).x - uncorrected.points.at(x).x;

        if (xdiff != 0)
            slopes.push_back(ydiff / xdiff);
    }//end for x.

    return getAverage(0, slopes.size() - 1, slopes);
}//get the average slope. 

void Shape::setSlope(double x)
{
    /**
      * When we set the slope to another number, we need
      * to correct the y-intercept (b) parameter as well!
     **/

    //y = mx + b -> y - mx = b
    double py = correctedSegment.points.at(0).y;
    double px = correctedSegment.points.at(0).x;

    S_m = x;
    S_b = py - S_m * px;
    updateSegment();
}//update the slope!

PointCloud Shape::getCorrections()
{
    if ((match_circle() < R2TOLL && match_segment() < R2TOLL) || 
        (match_circle() > R2TOLL2 && match_segment() > R2TOLL2))
        return uncorrected; //if not a shape, don't make one!

    if (is_circle() && C_r < MAXRAD)
        return correctedCircle;

    else
        return correctedSegment;
}//get the correct shape for the cloud handler.

void Shape::correct_circle()
{
    QList<pcl::PointXYZ> newPoints;

    QList<double> yVals;

    for (unsigned int x = 0; x < uncorrected.points.size(); x++)
        yVals.push_back(uncorrected.points.at(x).y);

    unsigned int min = getMindex(yVals);

    float a = uncorrected.points.front().x;
    float b = uncorrected.points.front().y;
    float c = uncorrected.points.at(min).x;
    float d = uncorrected.points.at(min).y;
    float e = uncorrected.points.back().x;
    float f = uncorrected.points.back().y;

    if (min == 0 || min == uncorrected.points.size() - 1)
    {
        //ROS_INFO("WARNING: Middle point is undefined.");
        c = (a + e) / 2.0;
        d = (b + f) / 2.0;
    }//fi

    //get three points from the scan.

    if ((b * (e - c)+ d * (a - e) + f*(c - a)) == 0)
    {
        while ((b * (e - c)+ d * (a - e) + f*(c - a)) == 0 && min < uncorrected.points.size())
        {
            //ROS_INFO("WARNING: NEW POINT BEING SELECTED");
            c = uncorrected.points.at(min).x;
            d = uncorrected.points.at(min).y;
            min++;
        }//end while
    }//end if

    float k = (1/2.0) * ((pow(a, 2) + pow(b, 2)) * (e - c) + (pow(c, 2)+pow(d, 2)) * (a-e) + (pow(e, 2)+pow(f, 2)) * (c-a)) / (b * (e - c)+ d * (a - e) + f*(c - a));
    float h = (0.5) * ((pow(a, 2) + pow(b, 2)) * (f - d) + (pow(c, 2) + pow(d, 2)) * (b - f) + (pow(e, 2) + pow(f, 2)) * (d - b)) / (a * (f - d) + c * (b - f) + e * (d - b));

    C_k = k;
    C_h = h;

    float r = sqrt(pow((a - h),2) + pow((b - k),2));
    C_r = r;
    
    for (unsigned int iter = 0; iter < uncorrected.points.size(); ++iter)
    {
        float x = uncorrected.points.at(iter).x;
        float y = uncorrected.points.at(iter).y;

        float possible_a = k + sqrt(pow(r, 2) - pow(x, 2) + 2 * h * x - pow(h, 2));
        float possible_b = k - sqrt(pow(r, 2) - pow(x, 2) + 2 * h * x - pow(h, 2));

        float a = abs(y - possible_a);
        float b = abs(y - possible_b);
        float radical = sqrt(pow(r, 2) - pow(x, 2) + 2 * h * x - pow(h, 2));

        if (a > b)
            radical *= -1;
        pcl::PointXYZ thePoint;
        thePoint.x = x; thePoint.y = k + radical; thePoint.z = Z;
        
        if (sqrt(pow(thePoint.y - uncorrected.points.at(iter).y, 2)) >= DIST_TOL)
        	thePoint.y = uncorrected.points.at(iter).y;
        newPoints.push_back(thePoint);
    }//end for

    pcl::PointCloud<pcl::PointXYZ> circlized;
    circlized.header.frame_id = "XY_Z_is1";
    circlized.header.stamp = ros::Time::now();

    for (int x = 0; x < newPoints.size(); ++x)
        circlized.points.push_back(newPoints.at(x));

    correctedCircle = circlized;
}//correct scan for a circle case 

void Shape::correct_segment()
{
    std::vector<pcl::PointXYZ> newPoints;

    /** Now we have the max and the min on the interval above! **/
    //y = mx + b -> y - mx = b
    bool undefined = false;
    float slope = ((double) uncorrected.points.at(uncorrected.points.size() - 1).y - uncorrected.points.at(0).y) /
            ((double) uncorrected.points.at(uncorrected.points.size() - 1).x - uncorrected.points.at(0).x) ;

    if (isnan(slope))
        undefined = true; // arbitrary constant
    float b = uncorrected.points.at(0).y - slope * uncorrected.points.at(0).y;

    for (unsigned int iter = 0; iter < uncorrected.points.size(); ++iter)
    {
        float px; float py;

        px = uncorrected.points.at(iter).x;
        py = slope * px + b;

        if (undefined)
        {
            px = uncorrected.points.at(0).x;
            py = uncorrected.points.at(iter).y;
        }//case of an undefined slope.

        pcl::PointXYZ toPush;
        toPush.x = px; toPush.y = py; toPush.z = Z;
        
        if (sqrt( pow(px - uncorrected.points.at(iter).x, 2) + pow(py - uncorrected.points.at(iter).y, 2)) > DIST_TOL)
        {	px = uncorrected.points.at(iter).x; py = uncorrected.points.at(iter).y; }

        newPoints.push_back(toPush);
    }//end for.
    pcl::PointXYZ orgin;
    orgin.x = 0; orgin.y = 0; orgin.z = 0;

    S_m = slope;
    S_b = b;

    PointCloud Segmetized;
    Segmetized.header.frame_id = "SegmentCloud";
    Segmetized.header.stamp = ros::Time::now();


    for (unsigned int x = 0; x < newPoints.size(); ++x)
        Segmetized.points.push_back(newPoints.at(x));

    correctedSegment = Segmetized;
}//correct as a segment

void Shape::updateSegment()
{
    std::vector<pcl::PointXYZ> newPoints;

    for (unsigned int iter = 0; iter < uncorrected.points.size(); ++iter)
    {
        float px = correctedSegment.points.at(iter).x;
        float py = S_m * px + S_b;
        pcl::PointXYZ toPush;
        toPush.x = px; toPush.y = py; toPush.z = Z;

        newPoints.push_back(toPush);
    }//end for.

    PointCloud Segmetized;
    Segmetized.header.frame_id = "XY_Z_is1";
    Segmetized.header.stamp = ros::Time::now();


    for (unsigned int x = 0; x < newPoints.size(); ++x)
        Segmetized.points.push_back(newPoints.at(x));

    correctedSegment = Segmetized;
}

double Shape::meanY()
{
    double sum = 0;

    for (unsigned int x = 0; x < uncorrected.points.size(); x++)
        sum += uncorrected.points.at(x).y;

    return sum / uncorrected.points.size();
}//get average y value

double Shape::match_circle()
{
    double sum1 = 0;
    double sum2 = 0;
    double yMean = meanY();

    for (unsigned int x = 0; x < uncorrected.points.size(); x++)
        sum1 += pow(uncorrected.points.at(x).y - yMean, 2);

    for (unsigned int y = 0; y < correctedCircle.points.size(); y++)
        sum2 += pow(correctedCircle.points.at(y).y - yMean, 2);

    return 1 - (sum1 / sum2);
}//r^2 value for the circle

double Shape::match_segment()
{
    double sum1 = 0;
    double sum2 = 0;
    double yMean = meanY();

    for (unsigned int x = 0; x < uncorrected.points.size(); x++)
        sum1 += pow(uncorrected.points.at(x).y - yMean, 2);

    for (unsigned int y = 0; y < correctedSegment.points.size(); y++)
        sum2 += pow(correctedSegment.points.at(y).y - yMean, 2);

    return 1 - (sum1 / sum2);
}//match for segment

bool Shape::is_circle()
{ return match_circle() >= match_segment(); }

bool Shape::is_segment()
{ return match_segment() > match_circle(); }

double Shape::getSlope()
{ return S_m; }//get the slope
