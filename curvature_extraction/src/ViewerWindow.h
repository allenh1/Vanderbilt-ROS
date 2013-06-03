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

private:

    QVBoxLayout *mainLayout;
    QPushButton *p_closeButton;
    QPushButton *p_saveButton;

    RobotThread m_RobotThread;
};

}//namespace server
#endif
