#include "ShapeAverageThread.h"
#include <vector>
#include <ros/ros.h>
#include <QVector>
namespace data_server {

ShapeAverageThread::ShapeAverageThread()
    : m_circleCount(0),
      m_shapeCount(0),
      m_segmentCount(0),
      m_bezierCount(0),
      alive(true)
{}

ShapeAverageThread::~ShapeAverageThread()
{
    alive = false;
    wait();
}

void ShapeAverageThread::pushCircle(double circles){ circleCounts.push_back(circles); }
void ShapeAverageThread::pushShape(double shapes){ shapeCounts.push_back(shapes); }
void ShapeAverageThread::pushSegment(double segment){ segmentCounts.push_back(segment); }
void ShapeAverageThread::pushCurve(double curves){ curveCounts.push_back(curves); }

double ShapeAverageThread::getAverage(QList<double> list)
{
    //ROS_INFO("Call to getAverage");

    int sum = 0;

    for (int x = 0; x < list.size(); x++)
    {
        //wait();
        int toAdd = list.at(x);
        sum += toAdd;
    }

    if (list.size() != 0)
        return ((double) sum ) / ((double) list.size());
    else
        return -1;
}

void ShapeAverageThread::averageShapes()
{
    m_shapeCount = getAverage(shapeCounts);
    Q_EMIT newShapeCount();
}//end void

void ShapeAverageThread::averageCircles()
{
    m_circleCount = getAverage(circleCounts);
    Q_EMIT newCircle();
}//average the circle count

void ShapeAverageThread::averageSegments()
{
    m_segmentCount = getAverage(segmentCounts);
    Q_EMIT newSegment();
}

void ShapeAverageThread::averageCurves()
{
    m_bezierCount = getAverage(curveCounts);
    Q_EMIT newCurve();
}

void ShapeAverageThread::run()
{
    while (alive)
    {
        if (!TryLock(100))
        {
            averageShapes();
            averageCircles();
            averageSegments();
            averageCurves();
        }//end if
        UnlockMutex();
    }
}

const double & ShapeAverageThread::getCircleCount(){ return m_circleCount; }
const double & ShapeAverageThread::getSegmentCount(){ return m_segmentCount; }
const double & ShapeAverageThread::getBezierCount(){ return m_bezierCount; }
const double & ShapeAverageThread::getShapeCount(){ return m_shapeCount; }
}//end namespace

