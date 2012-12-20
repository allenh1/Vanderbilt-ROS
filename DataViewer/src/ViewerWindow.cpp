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
    p_pointLayout = new QHBoxLayout();
    p_circleLayout = new QHBoxLayout();
    p_segmentLayout = new QHBoxLayout();
    p_bezierLayout = new QHBoxLayout();
    p_shapeLayout = new QHBoxLayout();

    p_shapeLabel = new QLabel();
    p_shapeLabel->setText("Average Shapes: ");
    p_shapeDisplay = new QLineEdit();
    p_shapeDisplay->setText("0.0");

    p_pointLabel = new QLabel();
    p_pointLabel->setText("Average Points: ");
    p_pointDisplay = new QLineEdit();
    p_pointDisplay->setText("0.0");

    p_circleLabel = new QLabel();
    p_circleLabel->setText("Average  Circles:");
    p_circleDisplay = new QLineEdit();
    p_circleDisplay->setText("0.0");

    p_segmentLabel = new QLabel();
    p_segmentLabel->setText("Average Segments:");
    p_segmentDisplay = new QLineEdit();
    p_segmentDisplay->setText("0.0");

    p_bezierLabel = new QLabel();
    p_bezierLabel->setText("Average Bezier: ");
    p_bezierDisplay = new QLineEdit();
    p_bezierDisplay->setText("0.0");

    p_circleLayout->addWidget(p_circleLabel);
    p_circleLayout->addWidget(p_circleDisplay);
    p_segmentLayout->addWidget(p_segmentLabel);
    p_segmentLayout->addWidget(p_segmentDisplay);
    p_bezierLayout->addWidget(p_bezierLabel);
    p_bezierLayout->addWidget(p_bezierDisplay);
    p_shapeLayout->addWidget(p_shapeLabel);
    p_shapeLayout->addWidget(p_shapeDisplay);
    p_pointLayout->addWidget(p_pointLabel);
    p_pointLayout->addWidget(p_pointDisplay);

    leftLayout->addLayout(p_shapeLayout);
    leftLayout->addLayout(p_pointLayout);
    leftLayout->addLayout(p_circleLayout);
    leftLayout->addLayout(p_segmentLayout);
    leftLayout->addLayout(p_bezierLayout);

    mainLayout = new QVBoxLayout();
    mainLayout->addLayout(leftLayout);
    mainLayout->addWidget(p_closeButton);
    setLayout(mainLayout);

    setWindowTitle(tr("Control Window"));

    connect(p_closeButton, SIGNAL(clicked()), this, SLOT(close()));
    connect(&m_RobotThread, SIGNAL(newCircle()), this, SLOT(updateCircleDisplay()));
    connect(&m_RobotThread, SIGNAL(newPoint()), this, SLOT(updatePointDisplay()));
    connect(&m_RobotThread, SIGNAL(newShapeCount()), this, SLOT(updateShapeDisplay()));
    connect(&m_RobotThread, SIGNAL(newSegment()), this, SLOT(updateSegmentDisplay()));
    connect(&m_RobotThread, SIGNAL(newCurve()), this, SLOT(updateCurveDisplay()));

    m_RobotThread.init();
    m_RobotThread.start();
}//end constructor

void ViewerWindow::updateShapeDisplay()
{
    QString shapeNum;
    shapeNum.setNum(m_RobotThread.getShapeCount());
    p_shapeDisplay->setText(shapeNum);
}

void ViewerWindow::updatePointDisplay()
{
    QString pointNum;
    pointNum.setNum(m_RobotThread.getPointCount());
    p_pointDisplay->setText(pointNum);
}

void ViewerWindow::updateCircleDisplay()
{
    QString circleNum;
    circleNum.setNum(m_RobotThread.getCircleCount());
    p_circleDisplay->setText(circleNum);
}

void ViewerWindow::updateSegmentDisplay()
{
    QString segmentNum;
    segmentNum.setNum(m_RobotThread.getSegmentCount());
    p_segmentDisplay->setText(segmentNum);
}

void ViewerWindow::updateCurveDisplay()
{
    QString curveNum;
    double curveNumber = m_RobotThread.getBezierCount();
    curveNum.setNum(curveNumber);
    p_bezierDisplay->setText(curveNum);
}
}//namespace server
