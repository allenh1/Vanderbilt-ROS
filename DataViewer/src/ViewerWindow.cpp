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

    p_minLabel = new QLabel();
    p_minLabel->setText("Minimum");
    p_averageLabel = new QLabel();
    p_averageLabel->setText("Mean");
    p_topLayout->addWidget(p_minLabel);
    p_topLayout->addWidget(p_averageLabel);

    p_shapeLabel = new QLabel();
    p_shapeLabel->setText("Shapes: ");
    p_shapeDisplay = new QLineEdit();
    p_shapeDisplay->setText("0.0");
    p_minShapes = new QLineEdit();
    p_minShapes->setText("0.0");

    p_pointLabel = new QLabel();
    p_pointLabel->setText("Points: ");
    p_pointDisplay = new QLineEdit();
    p_pointDisplay->setText("0.0");
    p_minPoints = new QLineEdit();
    p_minPoints->setText("0.0");

    p_circleLabel = new QLabel();
    p_circleLabel->setText("Circles:");
    p_circleDisplay = new QLineEdit();
    p_circleDisplay->setText("0.0");
    p_minCircle = new QLineEdit();
    p_minCircle->setText("0.0");

    p_segmentLabel = new QLabel();
    p_segmentLabel->setText("Segments:");
    p_segmentDisplay = new QLineEdit();
    p_segmentDisplay->setText("0.0");
    p_minSeg = new QLineEdit();
    p_minSeg->setText("0.0");

    p_bezierLabel = new QLabel();
    p_bezierLabel->setText("Bezier: ");
    p_bezierDisplay = new QLineEdit();
    p_bezierDisplay->setText("0.0");
    p_minBezier = new QLineEdit();
    p_minBezier->setText("0.0");

    p_timeLabel = new QLabel();
    p_timeLabel->setText("Time: ");
    p_timeDisplay = new QLineEdit();
    p_timeDisplay->setText("0.0");


    p_segErrorLabel = new QLabel();
    p_segErrorLabel->setText("Segment Error: ");
    p_segErrorDisplay = new QLineEdit();
    p_segErrorDisplay->setText("0.0");
    p_minError = new QLineEdit();
    p_minError->setText("0.0");

    p_circleLayout->addWidget(p_circleLabel);
    p_circleLayout->addWidget(p_minCircle);
    p_circleLayout->addWidget(p_circleDisplay);
    p_segmentLayout->addWidget(p_segmentLabel);
    p_segmentLayout->addWidget(p_minSeg);
    p_segmentLayout->addWidget(p_segmentDisplay);
    p_bezierLayout->addWidget(p_bezierLabel);
    p_bezierLayout->addWidget(p_minBezier);
    p_bezierLayout->addWidget(p_bezierDisplay);
    p_shapeLayout->addWidget(p_shapeLabel);
    p_shapeLayout->addWidget(p_minShapes);
    p_shapeLayout->addWidget(p_shapeDisplay);
    p_pointLayout->addWidget(p_pointLabel);
    p_pointLayout->addWidget(p_minPoints);
    p_pointLayout->addWidget(p_pointDisplay);
    p_timeLayout->addWidget(p_timeLabel);
    p_timeLayout->addWidget(p_timeDisplay);
    p_segErrorLayout->addWidget(p_segErrorLabel);
    p_segErrorLayout->addWidget(p_segErrorDisplay);

    leftLayout->addLayout(p_topLayout);
    leftLayout->addLayout(p_shapeLayout);
    leftLayout->addLayout(p_pointLayout);
    leftLayout->addLayout(p_circleLayout);
    leftLayout->addLayout(p_segmentLayout);
    leftLayout->addLayout(p_segErrorLayout);
    leftLayout->addLayout(p_bezierLayout);
    leftLayout->addLayout(p_timeLayout);

    mainLayout = new QVBoxLayout();
    mainLayout->addLayout(leftLayout);
    mainLayout->addWidget(p_closeButton);
    setLayout(mainLayout);

    setWindowTitle(tr("Data Window"));

    connect(p_closeButton, SIGNAL(clicked()), this, SLOT(close()));
    connect(&m_RobotThread, SIGNAL(newCircle()), this, SLOT(updateCircleDisplay()));
    connect(&m_RobotThread, SIGNAL(newPoint()), this, SLOT(updatePointDisplay()));
    connect(&m_RobotThread, SIGNAL(newShapeCount()), this, SLOT(updateShapeDisplay()));
    connect(&m_RobotThread, SIGNAL(newSegment()), this, SLOT(updateSegmentDisplay()));
    connect(&m_RobotThread, SIGNAL(newCurve()), this, SLOT(updateCurveDisplay()));
    connect(&m_RobotThread, SIGNAL(newTime()), this, SLOT(updateTimeDisplay()));
    connect(&m_RobotThread, SIGNAL(newMaxSegError()), this, SLOT(updateMaxSegErrorDisplay()));

    m_RobotThread.init();
    m_RobotThread.start();
}//end constructor

void ViewerWindow::updateShapeDisplay()
{
    QString shapeNum, minNum;
    minNum.setNum(m_RobotThread.getMinShapes());
    shapeNum.setNum(m_RobotThread.getShapeCount());
    p_minShapes->setText(minNum);
    p_shapeDisplay->setText(shapeNum);
}

void ViewerWindow::updatePointDisplay()
{
    QString pointNum, minNum;
    minNum.setNum(m_RobotThread.getMinPoints());
    pointNum.setNum(m_RobotThread.getPointCount());
    p_minPoints->setText(minNum);
    p_pointDisplay->setText(pointNum);
}

void ViewerWindow::updateCircleDisplay()
{
    QString circleNum, minNum;
    circleNum.setNum(m_RobotThread.getCircleCount());
    minNum.setNum(m_RobotThread.getMinCircles());
    p_circleDisplay->setText(circleNum);
    p_minCircle->setText(minNum);
}

void ViewerWindow::updateSegmentDisplay()
{
    QString segmentNum, minNum;
    segmentNum.setNum(m_RobotThread.getSegmentCount());
    minNum.setNum(m_RobotThread.getMinSegments());
    p_segmentDisplay->setText(segmentNum);
    p_minSeg->setText(minNum);
}

void ViewerWindow::updateCurveDisplay()
{
    QString curveNum, minNum;
    double curveNumber = m_RobotThread.getBezierCount();
    minNum.setNum(m_RobotThread.getMinCurves());
    curveNum.setNum(curveNumber);
    p_bezierDisplay->setText(curveNum);
    p_minBezier->setText(minNum);
}

void ViewerWindow::updateTimeDisplay()
{
    QString timeNum;
    double timeNumber = m_RobotThread.getTime();

    timeNum.setNum(timeNumber);
    p_timeDisplay->setText(timeNum);
}

void ViewerWindow::updateMaxSegErrorDisplay()
{
    QString errorNum;
    errorNum.setNum(m_RobotThread.getMaxSegError());

    p_segErrorDisplay->setText(errorNum);
}
}//namespace server
