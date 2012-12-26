#include "ShapeAverageThread.h"
#include <vector>
#include <ros/ros.h>
#include <QVector>
namespace data_server {

ShapeAverageThread::ShapeAverageThread()
    : m_circleCount(0),
      m_pointCount(0),
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
void ShapeAverageThread::pushPoint(double points){ pointCounts.push_back(points); }

void ShapeAverageThread::pushTime(double time)
{
    double delta_t = time - m_lastTime;

    if (m_lastTime < 0)
        delta_t = 0;

    timeDiffs.push_back(delta_t);

    m_lastTime = time;
}//add the last time to the array.

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

void ShapeAverageThread::averagePoints()
{
    m_pointCount = getAverage(pointCounts);
    Q_EMIT newPoint();
}

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

void ShapeAverageThread::averageTimes()
{
    m_Time = getAverage(timeDiffs);
    Q_EMIT newTimeDiff();
}

void ShapeAverageThread::run()
{
    m_lastTime = -1;

    while (alive)
    {
        if (!TryLock(1000))
        {
            averageShapes();
            averagePoints();
            averageCircles();
            averageSegments();
            averageCurves();
            averageTimes();
        }//end if
        UnlockMutex();
    }
}

const double & ShapeAverageThread::getCircleCount(){ return m_circleCount; }
const double & ShapeAverageThread::getPointCount(){ return m_pointCount; }
const double & ShapeAverageThread::getSegmentCount(){ return m_segmentCount; }
const double & ShapeAverageThread::getBezierCount(){ return m_bezierCount; }
const double & ShapeAverageThread::getShapeCount(){ return m_shapeCount; }
const double & ShapeAverageThread::getTimeDiff(){ return m_Time; }
}//end namespace

