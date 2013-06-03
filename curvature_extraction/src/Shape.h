#ifndef SHAPE_H
#define SHAPE_H


#include <iostream>
#include <QFile>
#include <QTextStream>
#include <QMessageBox>
#include <QFileDialog>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <Eigen/StdVector>
#include <ros/ros.h>
#include <geometry_msgs/Point32.h>
#include "MathLibrary.cpp"

namespace data_server {

const float PHIe =  PI / 20.0; // ~ 10 degrees
const double SLOPE_TOLERATION = 0.1; // difference is less than 0.1
const double DIST_TOLERANCE = 0.2; //50 cm distance tolerance.
const double DIST_TOL = 0.1;
const double MAXRAD = 0.75; //maximum radius of .75 meters
const double MINRAD = 0.25;
const double R2TOLL = 0.9995;
const double R2TOLL2 = 1.01;


enum SHAPE_TYPE { CIRCLE = 1, SEGMENT = 2, BEZIER = 3, ORIG = 4 };

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

template<int N>
struct Factorial
{ enum{ val = N * Factorial<N - 1>::val }; };

template<>
struct Factorial<0>
{ enum{ val = 1 }; };

struct indexSegment
{
    double slope;
    unsigned int index;
};

const float min_dist = 0.7; //50 cm
//int asdf = 1;

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
    double max_correct;

    bool is_circle();
    bool is_bezier();
    bool is_segment();

    const SHAPE_TYPE & getType();

private:
    PointCloud uncorrected;
    PointCloud correctedSegment;
    PointCloud correctedCircle;
    PointCloud correctedCurve;

    void correct_circle();
    void correct_Bezier();
    void correct_segment();
    void updateSegment();

    SHAPE_TYPE shapeType;

    double match_circle();
    double match_segment();
    double match_bezier();
    double meanY();
    double getAverageSlope();

    /** This is the Slope Data **/

    double S_m; //slope of correction.
    double S_b; //intercept of correction.
    double C_k; //the k parameter of a circle.
    double C_h; //the h parameter of a circle.
    double C_r; //the radius of the circle

};
}
#endif
