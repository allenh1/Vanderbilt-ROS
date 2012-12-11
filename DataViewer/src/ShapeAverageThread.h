#ifndef ___SHAPEAVERAGETHREAD_H___
#define ___SHAPEAVERAGETHREAD_H___

#include <QThread>
#include <QObject>
#include <QMutex>
#include <QStringList>
#include <stdlib.h>
#include <iostream>
#include "assert.h"

namespace data_server {

class ShapeAverageThread : public QThread {
    Q_OBJECT
public:
    ShapeAverageThread();
    virtual ~ShapeAverageThread();

    const double & getCircleCount();
    const double & getSegmentCount();
    const double & getBezierCount();
    const double & getShapeCount();

    void run();

    void pushShape(double);
    void pushCircle(double);
    void pushSegment(double);
    void pushCurve(double);

    Q_SIGNAL void newCircle();
    Q_SIGNAL void newCurve();
    Q_SIGNAL void newSegment();
    Q_SIGNAL void newShapeCount();

    void LockMutex(){m_Mutex.lock();}
    void UnlockMutex(){m_Mutex.unlock();}
    bool TryLock(int timeOut = 0){if(timeOut) return m_Mutex.tryLock(); else return m_Mutex.tryLock(timeOut);}

private:
    void averageShapes();
    void averageSegments();
    void averageCircles();
    void averageCurves();

    QMutex m_Mutex;

    double getAverage(QList<double> list);

    double m_circleCount;
    double m_shapeCount;
    double m_segmentCount;
    double m_bezierCount;
    
    bool alive;

    QList<double> shapeCounts;
    QList<double> circleCounts;
    QList<double> segmentCounts;
    QList<double> curveCounts;
};

}//end namespace
#endif

