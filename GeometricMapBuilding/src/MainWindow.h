/**
 * @file /include/qdude/main_window.hpp
 *
 * @brief Qt based gui for qdude.
 *
 * @date November 2010
 **/
#ifndef qdude_MAIN_WINDOW_H
#define qdude_MAIN_WINDOW_H

/*****************************************************************************
** Includes
*****************************************************************************/

#include <QtGui/QMainWindow>
#include <QTimer>
#include "ui_MainWindow.h"
#include "RobotThread.h"
/*****************************************************************************
** Namespace
*****************************************************************************/

namespace server {

/*****************************************************************************
** Interface [MainWindow]
*****************************************************************************/
/**
 * @brief Qt central, all operations relating to the view part here.
 */
class MainWindow : public QMainWindow {
Q_OBJECT

public:
	MainWindow(int argc, char** argv, QWidget *parent = 0);
	~MainWindow();
    //const Foo& v
    Q_SLOT void updateDisplay();
    Q_SIGNAL void unlockPoint();

    void readFromRobot();
	void closeEvent(QCloseEvent *event); // Overloaded function

private:

    double m_lastRosTime;
    Ui::MainWindow ui;
    RobotThread robot;
    QTimer * m_pRefreshTimer;
};

}  // namespace qdude

#endif // qdude_MAIN_WINDOW_H
