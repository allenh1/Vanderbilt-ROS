#ifndef SERVER_H
#define SERVER_H

//ros include files
#include "ros/ros.h"
#include "std_msgs/String.h"

//Qt4 related includes
#include <QObject>
#include <QStringList>
#include <qt4/QtNetwork/QTcpServer>
#include <qt4/QtNetwork/QTcpSocket>

class Server
{
public:
    Server();
    void writeData();
    bool init();
    QString getCommand();

private:

    QTcpServer m_pTcpServer;
    std_msgs::String str;
    ros::Publisher pub;
};

#endif
