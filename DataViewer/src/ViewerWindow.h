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

namespace data_server {
class ViewerWindow : public QWidget
{
    Q_OBJECT

public:
    ViewerWindow(int argc, char** argv, QWidget * parent = 0);

    Q_SLOT void updateCircleDisplay();
    Q_SLOT void updateShapeDisplay();
    Q_SLOT void updateSegmentDisplay();

private:
    QVBoxLayout *leftLayout;
    QHBoxLayout *p_circleLayout;
    QHBoxLayout *p_segmentLayout;
    QHBoxLayout *p_bezierLayout;
    QHBoxLayout *p_shapeLayout;

    QLabel *p_circleLabel;
    QLabel *p_segmentLabel;
    QLabel *p_bezierLabel;
    QLabel *p_shapeLabel;

    QLineEdit *p_circleDisplay;
    QLineEdit *p_segmentDisplay;
    QLineEdit *p_bezierDisplay;
    QLineEdit *p_shapeDisplay;

    QVBoxLayout *mainLayout;
    QPushButton *p_closeButton;

    RobotThread m_RobotThread;
};

}//namespace server
#endif
