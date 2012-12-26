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

    Q_SLOT void updateCircleDisplay();
    Q_SLOT void updatePointDisplay();
    Q_SLOT void updateShapeDisplay();
    Q_SLOT void updateSegmentDisplay();
    Q_SLOT void updateCurveDisplay();
    Q_SLOT void updateTimeDisplay();

private:
    QVBoxLayout *leftLayout;
    QHBoxLayout *p_circleLayout;
    QHBoxLayout *p_pointLayout;
    QHBoxLayout *p_segmentLayout;
    QHBoxLayout *p_bezierLayout;
    QHBoxLayout *p_shapeLayout;
    QHBoxLayout *p_timeLayout;

    QLabel *p_circleLabel;
    QLabel *p_pointLabel;
    QLabel *p_segmentLabel;
    QLabel *p_bezierLabel;
    QLabel *p_shapeLabel;
    QLabel *p_timeLabel;

    QLineEdit *p_circleDisplay;
    QLineEdit *p_segmentDisplay;
    QLineEdit *p_pointDisplay;
    QLineEdit *p_bezierDisplay;
    QLineEdit *p_shapeDisplay;
    QLineEdit *p_timeDisplay;

    QVBoxLayout *mainLayout;
    QPushButton *p_closeButton;

    RobotThread m_RobotThread;
    ShapeAverageThread m_MathThread;
};

}//namespace server
#endif
