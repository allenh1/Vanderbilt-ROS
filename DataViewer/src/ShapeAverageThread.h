#ifndef ___SHAPEAVERAGETHREAD_H___
#define ___SHAPEAVERAGETHREAD_H___

#include <QThread>
#include <QObject>
#include <QMutex>
#include <QStringList>
#include <QtAlgorithms>
#include <stdlib.h>
#include <iostream>
#include "assert.h"

namespace data_server {

class ShapeAverageThread : public QThread {
    Q_OBJECT
public:
    ShapeAverageThread();
    virtual ~ShapeAverageThread();

    void run();

    Q_SLOT void pushShape(double);
    Q_SLOT void pushTime(double);
    Q_SLOT void pushPoint(double);
    Q_SLOT void pushCircle(double);
    Q_SLOT void pushSegment(double);
    Q_SLOT void pushCurve(double);
    Q_SLOT void pushSegError(double);

    Q_SIGNAL void newCircle(double, double, double);
    Q_SIGNAL void newTimeDiff(double, double, double);
    Q_SIGNAL void newPoint(double, double, double);
    Q_SIGNAL void newCurve(double, double, double);
    Q_SIGNAL void newSegment(double, double, double);
    Q_SIGNAL void newShapeCount(double, double, double,double);
    Q_SIGNAL void newMaxSegError(double, double);

    void LockMutex(){m_Mutex.lock();}
    void UnlockMutex(){m_Mutex.unlock();}
    bool TryLock(int timeOut = 0){if(timeOut) return m_Mutex.tryLock(timeOut); else return m_Mutex.tryLock();}

private:
    void averageShapes();
    void averageSegments();
    void averageTimes();
    void averagePoints();
    void averageCircles();
    void averageCurves();
    void setMaxError();

    QMutex m_Mutex;

    double getAverage(QList<double> &list);

    double m_circleCount;
    double m_pointCount;
    double m_shapeCount;
    double m_segmentCount;
    double m_bezierCount;
    double m_Time;
    double m_maxSegError;

    double m_lastTime;
    
    bool alive;

    QList<double> shapeCounts;
    QList<double> timeDiffs;
    QList<double> pointCounts;
    QList<double> circleCounts;
    QList<double> segmentCounts;
    QList<double> curveCounts;
    QList<double> maxSegError;
};

}//end namespace
#endif

