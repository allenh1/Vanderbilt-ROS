#ifndef SERVER_H
#define SERVER_H

#include "RobotThread.h"
#include <geometry_msgs/Point.h>
#include <QObject>

QT_BEGIN_NAMESPACE
class QTcpServer;
QT_END_NAMESPACE

namespace server {

#define PI 3.1415926535898

class Server : public QObject {
    Q_OBJECT

public:
    Server(int argc, char** argv, QObject* pParent = NULL);
    //~Server();

    void writeData();

private:

    RobotThread m_RobotThread;
};

}//end namespace
#endif
