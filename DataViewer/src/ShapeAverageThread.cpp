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
      m_maxSegError(0),
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
void ShapeAverageThread::pushSegError(double error){ maxSegError.push_back(error); }

void ShapeAverageThread::pushTime(double time)
{
    double delta_t = time - m_lastTime;

    if (m_lastTime < 0)
        delta_t = 0;

    timeDiffs.push_back(delta_t);

    m_lastTime = time;
}//add the last time to the array.

double ShapeAverageThread::getAverage(QList<double> & list)
{
    //ROS_INFO("Call to getAverage");

    int sum = 0;

    if (list.size() < 3)
        return -1;

    qSort(list.begin(), list.end());

    for (int x = 0; x < list.size(); x++)
    {
        //wait();
        int toAdd = list.at(x);
        sum += toAdd;
    }

    return ((double) sum ) / ((double) list.size());
}

void ShapeAverageThread::averageShapes()
{
    QList<double> theseShapes = shapeCounts;
    m_shapeCount = getAverage(theseShapes);
    if (m_shapeCount != -1)
        Q_EMIT newShapeCount(theseShapes.first(), m_shapeCount, theseShapes.last(), theseShapes.size());
}//end void

void ShapeAverageThread::averagePoints()
{
    QList<double> current = pointCounts;
    m_pointCount = getAverage(current);
    if (m_pointCount != -1)
        Q_EMIT newPoint(current.first(), m_pointCount, current.last());
}

void ShapeAverageThread::averageCircles()
{
    QList<double> current = circleCounts;
    m_circleCount = getAverage(current);
    if (m_circleCount != -1)
        Q_EMIT newCircle(current.first(), m_circleCount, current.last());
}//average the circle count

void ShapeAverageThread::averageSegments()
{
    QList<double> current = segmentCounts;
    m_segmentCount = getAverage(current);
    if (m_segmentCount != -1)
        Q_EMIT newSegment(current.first(), m_segmentCount, current.last());
}

void ShapeAverageThread::averageCurves()
{
    QList<double> current = curveCounts;
    m_bezierCount = getAverage(current);
    if (m_bezierCount != -1)
        Q_EMIT newCurve(current.first(), m_bezierCount, current.last());
}

void ShapeAverageThread::averageTimes()
{
    QList<double> current = timeDiffs;
    m_Time = getAverage(current);

    if (m_Time != -1)
        Q_EMIT(newTimeDiff(current.first(), m_Time, current.last()));
}

void ShapeAverageThread::setMaxError()
{
    if (maxSegError.size() > 0)
    {
        QList<double> current = maxSegError;
        qSort(current.begin(), current.end());

        Q_EMIT(newMaxSegError(current.first(), current.last()));
    }//don't update if empty!
}//end void

void ShapeAverageThread::run()
{
    m_lastTime = -1;

    while (alive)
    {
        averageShapes();
        averagePoints();
        averageCircles();
        averageSegments();
        averageCurves();
        averageTimes();
        setMaxError();
    }//end while
}//run the thread!
}//end namespace

