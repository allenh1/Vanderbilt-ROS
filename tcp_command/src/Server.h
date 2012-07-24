#ifndef SERVER_H
#define SERVER_H

#include <QObject>
#include <QStringList>

#include "RobotThread.h"

QT_BEGIN_NAMESPACE
class QTcpServer;
QT_END_NAMESPACE

class Server : public QObject {
    Q_OBJECT

public:
    Server(int argc,char ** argv, QObject* pParent = NULL);
    virtual ~Server();

    void writeData();

private Q_SLOTS:
	void NewClientConnection();
	void NewClientCommand();

private:
    QTcpServer* m_pTcpServer;

    RobotThread m_RobotThread;
};

#endif
