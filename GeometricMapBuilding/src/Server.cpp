#include <qt4/QtNetwork/QtNetwork>

#include "Server.h"
namespace server {

Server::Server(int argc, char** argv, QObject* pParent)
    :	QObject(pParent),
      m_RobotThread(argc, argv)
{
    m_RobotThread.init();
    m_RobotThread.start();
}//the server starts the RobotThread.

}//namespace
