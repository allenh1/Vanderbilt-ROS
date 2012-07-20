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

QT_BEGIN_NAMESPACE
class QTcpServer;
QT_END_NAMESPACE

class Server : public QObject {
    Q_OBJECT

public:
    Server(int argc, char** argv, QObject* pParent = NULL);
    virtual ~Server();
    bool init();

private Q_SLOTS:
	void NewClientConnection();
	void NewClientCommand();

private:
    int init_argc;
    char** init_argv;

    QTcpServer* m_pTcpServer;
    std_msgs::String str;
    ros::Publisher pub;
};

#endif
