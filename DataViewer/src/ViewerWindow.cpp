#include "ViewerWindow.h"
#include <QPalette>
#define PI 3.1415926535898

namespace data_server{
ViewerWindow::ViewerWindow(int argc, char **argv, QWidget *parent)
    : QWidget(parent),
      m_RobotThread(argc, argv),
      m_MathThread()
{
    /** Set up the Controls **/
    p_closeButton = new QPushButton(tr("&Quit"));

    /** Set up the Position Display **/
    leftLayout = new QVBoxLayout();
    p_topLayout = new QHBoxLayout();
    p_pointLayout = new QHBoxLayout();
    p_circleLayout = new QHBoxLayout();
    p_segmentLayout = new QHBoxLayout();
    p_bezierLayout = new QHBoxLayout();
    p_shapeLayout = new QHBoxLayout();
    p_timeLayout = new QHBoxLayout();
    p_segErrorLayout = new QHBoxLayout();
    p_dataLayout = new QHBoxLayout();

    p_typeLabel = new QLabel();
    p_typeLabel->setText("Type");
    p_maxLabel = new QLabel();
    p_maxLabel->setText("Maximum");
    p_minLabel = new QLabel();
    p_minLabel->setText("Minimum");
    p_averageLabel = new QLabel();
    p_averageLabel->setText("Mean");
    p_topLayout->addWidget(p_typeLabel);
    p_topLayout->addWidget(p_minLabel);
    p_topLayout->addWidget(p_averageLabel);
    p_topLayout->addWidget(p_maxLabel);

    p_shapeLabel = new QLabel();
    p_shapeLabel->setText("Shapes: ");
    p_shapeDisplay = new QLineEdit();
    p_shapeDisplay->setText("0.0");
    p_minShapes = new QLineEdit();
    p_minShapes->setText("0.0");
    p_maxShapes = new QLineEdit();
    p_maxShapes->setText("0.0");

    p_pointLabel = new QLabel();
    p_pointLabel->setText("Points: ");
    p_pointDisplay = new QLineEdit();
    p_pointDisplay->setText("0.0");
    p_minPoints = new QLineEdit();
    p_minPoints->setText("0.0");
    p_maxPoints = new QLineEdit();
    p_maxPoints->setText("0.0");

    p_circleLabel = new QLabel();
    p_circleLabel->setText("Circles:");
    p_circleDisplay = new QLineEdit();
    p_circleDisplay->setText("0.0");
    p_minCircle = new QLineEdit();
    p_minCircle->setText("0.0");
    p_maxCircle = new QLineEdit();
    p_maxCircle->setText("0.0");

    p_segmentLabel = new QLabel();
    p_segmentLabel->setText("Segments:");
    p_segmentDisplay = new QLineEdit();
    p_segmentDisplay->setText("0.0");
    p_minSeg = new QLineEdit();
    p_minSeg->setText("0.0");
    p_maxSeg = new QLineEdit();
    p_maxSeg->setText("0.0");

    p_bezierLabel = new QLabel();
    p_bezierLabel->setText("Bezier: ");
    p_bezierDisplay = new QLineEdit();
    p_bezierDisplay->setText("0.0");
    p_minBezier = new QLineEdit();
    p_minBezier->setText("0.0");
    p_maxBezier = new QLineEdit();
    p_maxBezier->setText("0.0");

    p_timeLabel = new QLabel();
    p_timeLabel->setText("Time: ");
    p_timeDisplay = new QLineEdit();
    p_timeDisplay->setText("0.0");
    p_maxTime = new QLineEdit();
    p_minTime = new QLineEdit();
    p_maxTime->setText("0.0");
    p_minTime->setText("0.0");

    p_segErrorLabel = new QLabel();
    p_segErrorLabel->setText("Segment Error: ");
    p_segErrorDisplay = new QLineEdit();
    p_segErrorDisplay->setText("0.0");
    p_minError = new QLineEdit();
    p_minError->setText("0.0");
    p_maxError = new QLineEdit();
    p_maxError->setText("0.0");

    p_dataLabel = new QLabel();
    p_dataLabel->setText("Number Of Data Points: ");
    p_dataSize = new QLineEdit();
    p_dataSize->setText("0");

    p_dataLayout->addWidget(p_dataLabel);
    p_dataLayout->addWidget(p_dataSize);
    p_circleLayout->addWidget(p_circleLabel);
    p_circleLayout->addWidget(p_minCircle);
    p_circleLayout->addWidget(p_circleDisplay);
    p_circleLayout->addWidget(p_maxCircle);
    p_segmentLayout->addWidget(p_segmentLabel);
    p_segmentLayout->addWidget(p_minSeg);
    p_segmentLayout->addWidget(p_segmentDisplay);
    p_segmentLayout->addWidget(p_maxSeg);
    p_bezierLayout->addWidget(p_bezierLabel);
    p_bezierLayout->addWidget(p_minBezier);
    p_bezierLayout->addWidget(p_bezierDisplay);
    p_bezierLayout->addWidget(p_maxBezier);
    p_shapeLayout->addWidget(p_shapeLabel);
    p_shapeLayout->addWidget(p_minShapes);
    p_shapeLayout->addWidget(p_shapeDisplay);
    p_shapeLayout->addWidget(p_maxShapes);
    p_pointLayout->addWidget(p_pointLabel);
    p_pointLayout->addWidget(p_minPoints);
    p_pointLayout->addWidget(p_pointDisplay);
    p_pointLayout->addWidget(p_maxPoints);
    p_timeLayout->addWidget(p_timeLabel);
    p_timeLayout->addWidget(p_minTime);
    p_timeLayout->addWidget(p_timeDisplay);
    p_timeLayout->addWidget(p_maxTime);
    p_segErrorLayout->addWidget(p_segErrorLabel);
    p_segErrorLayout->addWidget(p_segErrorDisplay);
    p_segErrorLayout->addWidget(p_maxError);

    leftLayout->addLayout(p_topLayout);
    leftLayout->addLayout(p_shapeLayout);
    leftLayout->addLayout(p_pointLayout);
    leftLayout->addLayout(p_circleLayout);
    leftLayout->addLayout(p_segmentLayout);
    leftLayout->addLayout(p_segErrorLayout);
    leftLayout->addLayout(p_bezierLayout);
    leftLayout->addLayout(p_timeLayout);
    leftLayout->addLayout(p_dataLayout);

    mainLayout = new QVBoxLayout();
    mainLayout->addLayout(leftLayout);
    mainLayout->addWidget(p_closeButton);
    setLayout(mainLayout);

    setWindowTitle(tr("Data Window"));

    connect(p_closeButton, SIGNAL(clicked()), this, SLOT(close()));

    connect(&m_MathThread, SIGNAL(newCircle(double,double,double)), this, SLOT(updateCircleDisplay(double,double,double)));
    connect(&m_MathThread, SIGNAL(newPoint(double,double,double)), this, SLOT(updatePointDisplay(double,double,double)));
    connect(&m_MathThread, SIGNAL(newShapeCount(double, double, double,double)), this, SLOT(updateShapeDisplay(double,double,double,double)));
    connect(&m_MathThread, SIGNAL(newSegment(double, double, double)), this, SLOT(updateSegmentDisplay(double,double,double)));
    connect(&m_MathThread, SIGNAL(newCurve(double,double,double)), this, SLOT(updateCurveDisplay(double,double,double)));
    connect(&m_MathThread, SIGNAL(newMaxSegError(double,double)), this, SLOT(updateMaxSegErrorDisplay(double,double)));
    connect(&m_MathThread, SIGNAL(newTimeDiff(double,double,double)), this, SLOT(updateTimeDisplay(double,double,double)));

    connect(&m_RobotThread, SIGNAL(CircleInformation(double)), &m_MathThread, SLOT(pushCircle(double)));
    connect(&m_RobotThread, SIGNAL(SegmentInformation(double)), &m_MathThread, SLOT(pushSegment(double)));
    connect(&m_RobotThread, SIGNAL(PointInformation(double)), &m_MathThread, SLOT(pushPoint(double)));
    connect(&m_RobotThread, SIGNAL(BezierInformation(double)), &m_MathThread, SLOT(pushCurve(double)));
    connect(&m_RobotThread, SIGNAL(ShapeInformation(double)), &m_MathThread, SLOT(pushShape(double)));
    connect(&m_RobotThread, SIGNAL(TimeInformation(double)), &m_MathThread, SLOT(pushTime(double)));
    connect(&m_RobotThread, SIGNAL(SegmentErrorInformation(double)), &m_MathThread, SLOT(pushSegError(double)));

    m_RobotThread.init();
    m_RobotThread.start();
    m_MathThread.start();
}//end constructor

void ViewerWindow::updateShapeDisplay(double min, double num, double max, double dataSize)
{
    QString shapeNum, minNum, maxNum, dataNum;
    minNum.setNum(min);
    dataNum.setNum(dataSize);
    maxNum.setNum(max);
    shapeNum.setNum(num);
    p_minShapes->setText(minNum);
    p_shapeDisplay->setText(shapeNum);
    p_maxShapes->setText(maxNum);
    p_dataSize->setText(dataNum);
}

void ViewerWindow::updatePointDisplay(double min, double num, double max)
{
    QString pointNum, minNum, maxNum;
    minNum.setNum(min);
    pointNum.setNum(num);
    maxNum.setNum(max);
    p_minPoints->setText(minNum);
    p_pointDisplay->setText(pointNum);
    p_maxPoints->setText(maxNum);
}

void ViewerWindow::updateCircleDisplay(double min, double num, double max)
{
    QString circleNum, minNum, maxNum;
    circleNum.setNum(num);
    minNum.setNum(min);
    maxNum.setNum(max);
    p_circleDisplay->setText(circleNum);
    p_minCircle->setText(minNum);
    p_maxCircle->setText(maxNum);
}

void ViewerWindow::updateSegmentDisplay(double min, double num, double max)
{
    QString segmentNum, minNum, maxNum;
    segmentNum.setNum(num);
    minNum.setNum(min);
    maxNum.setNum(max);
    p_segmentDisplay->setText(segmentNum);
    p_minSeg->setText(minNum);
    p_maxSeg->setText(maxNum);
}

void ViewerWindow::updateCurveDisplay(double min, double num, double max)
{
    QString curveNum, minNum, maxNum;
    double curveNumber = num;
    minNum.setNum(min);
    maxNum.setNum(max);
    curveNum.setNum(curveNumber);
    p_bezierDisplay->setText(curveNum);
    p_minBezier->setText(minNum);
    p_maxBezier->setText(maxNum);
}

void ViewerWindow::updateTimeDisplay(double min, double num, double max)
{
    QString timeNum, minNum, maxNum;
    double timeNumber = num;

    timeNum.setNum(timeNumber, 'g', 15);
    minNum.setNum(min);
    maxNum.setNum(max);
    p_timeDisplay->setText(timeNum);
    p_minTime->setText(minNum);
    p_maxTime->setText(maxNum);
}

void ViewerWindow::updateMaxSegErrorDisplay(double min, double max)
{
    QString minError, maxError;
    minError.setNum(min);
    maxError.setNum(max);
    p_maxError->setText(maxError);
    p_minError->setText(minError);
}
}//namespace server
