#ifndef VIEWER_WINDOW_H
#define VIEWER_WINDOW_H
#include <QMainWindow>
#include <QList>
#include <QFileDialog>
#include <QMessageBox>
#include <QtAlgorithms>
#include <QPushButton>
#include <QString>
#include <QVBoxLayout>
#include <QHBoxLayout>
#include <QLabel>
#include <QTextStream>
#include <QLineEdit>
#include <QIcon>
#include "RobotThread.h"
#include "ShapeAverageThread.h"

namespace data_server {
class ViewerWindow : public QWidget
{
    Q_OBJECT

public:
    ViewerWindow(int argc, char** argv, QWidget * parent = 0);

    Q_SLOT void updateCircleDisplay(double min, double num, double max);
    Q_SLOT void updatePointDisplay(double min, double num, double max);
    Q_SLOT void updateShapeDisplay(double min, double num, double max, double dataSize);
    Q_SLOT void updateSegmentDisplay(double min, double num, double max);
    Q_SLOT void updateCurveDisplay(double min, double num, double max);
    Q_SLOT void updateTimeDisplay(double min, double num, double max);
    Q_SLOT void updateMaxSegErrorDisplay(double min, double max);

private:
    QVBoxLayout *leftLayout;
    QHBoxLayout *p_topLayout;
    QHBoxLayout *p_circleLayout;
    QHBoxLayout *p_pointLayout;
    QHBoxLayout *p_segmentLayout;
    QHBoxLayout *p_bezierLayout;
    QHBoxLayout *p_shapeLayout;
    QHBoxLayout *p_timeLayout;
    QHBoxLayout *p_segErrorLayout;
    QHBoxLayout *p_dataLayout;

    QLabel *p_typeLabel;
    QLabel *p_minLabel;
    QLabel *p_maxLabel;
    QLabel *p_averageLabel;
    QLabel *p_dataLabel;

    QLabel *p_circleLabel;
    QLabel *p_pointLabel;
    QLabel *p_segmentLabel;
    QLabel *p_bezierLabel;
    QLabel *p_shapeLabel;
    QLabel *p_timeLabel;
    QLabel *p_segErrorLabel;

    QLineEdit *p_minShapes;
    QLineEdit *p_minPoints;
    QLineEdit *p_minCircle;
    QLineEdit *p_minSeg;
    QLineEdit *p_minBezier;
    QLineEdit *p_minError;
    QLineEdit *p_minTime;

    QLineEdit *p_maxShapes;
    QLineEdit *p_maxPoints;
    QLineEdit *p_maxCircle;
    QLineEdit *p_maxSeg;
    QLineEdit *p_maxBezier;
    QLineEdit *p_maxError;
    QLineEdit *p_maxTime;

    QLineEdit *p_circleDisplay;
    QLineEdit *p_segmentDisplay;
    QLineEdit *p_pointDisplay;
    QLineEdit *p_bezierDisplay;
    QLineEdit *p_shapeDisplay;
    QLineEdit *p_timeDisplay;
    QLineEdit *p_segErrorDisplay;
    QLineEdit *p_dataSize;

    QVBoxLayout *mainLayout;
    QPushButton *p_closeButton;

    RobotThread m_RobotThread;
    ShapeAverageThread m_MathThread;
};

}//namespace server
#endif
