#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/LaserScan.h"
#include "laserHandle.h"
#include "RobotThread.h"
#include "MainWindow.h"
#include "Server.h"
#include <QCoreApplication>
#include <QApplication>

using namespace server;

int main(int argc, char** argv){
    QApplication app(argc, argv);
    server::MainWindow w(argc,argv);
    w.show();
    app.connect(&app, SIGNAL(lastWindowClosed()), &app, SLOT(quit()));
    //for(;;)
    //	;
    //system("pause");
    return app.exec();
}
