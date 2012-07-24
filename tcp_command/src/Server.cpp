#include <qt4/QtNetwork/QtNetwork>

#include <stdlib.h>
#include <iostream>
#include "assert.h"

#include "Server.h"

Server::Server(int argc, char **argv, QObject* pParent)
    :	QObject(pParent),
        m_RobotThread(argc, argv) {

    m_pTcpServer = new QTcpServer(this);
    if (!m_pTcpServer->listen(QHostAddress::Any, 5512)) {
		std::cerr << tr("TCP Server").toStdString(),
                              tr("Unable to start the server: %1.\n")
							  .arg(m_pTcpServer->errorString()).toStdString();
		exit(-1);
        return;
    }

    QString ipAddress = QHostAddress(QHostAddress::LocalHost).toString();
	std::cout << tr("The server is running on\n\nIP: %1\nport: %2\n\n"
                            "Run the Client now.")
							.arg(ipAddress).arg(m_pTcpServer->serverPort()).toStdString() << std::endl;

	connect(m_pTcpServer, SIGNAL(newConnection()), SLOT(NewClientConnection()));

	m_RobotThread.start();
}

Server::~Server()
{}

void Server::NewClientConnection() {
	QTcpSocket * pClientSocket = m_pTcpServer->nextPendingConnection();
	if(pClientSocket) {
		connect(pClientSocket, SIGNAL(disconnected()), pClientSocket, SLOT(deleteLater()));
		connect(pClientSocket, SIGNAL(readyRead()), SLOT(NewClientCommand()));
	}
}

void Server::writeData()
{
	QTcpSocket *pClientSocket = qobject_cast<QTcpSocket*>(sender());
	const QRegExp rxlen("^(\\w+)\\s+(-*\\d*\\.?\\d*)\\s+(-*\\d*\\.?\\d*)$");
//	QString text(pClientSocket->read(length));
	
//	pClientSocket->write(m_RobotThread.getForwardSpeed());
}

void Server::NewClientCommand() {
	// "\w": match a word character
	// "\s": match a whitespace character
	// "\d": match a digit
	const QRegExp rxlen("^(\\w+)\\s+(-*\\d*\\.?\\d*)\\s+(-*\\d*\\.?\\d*)$");
	// [word] [digit] 
	int length;

	QTcpSocket * pClientSocket = qobject_cast<QTcpSocket *>(sender());
	
	while(pClientSocket->bytesAvailable() > 0) {
		length = static_cast<int>(pClientSocket->read(1).at(0));	// Read the command size.
		QString text(pClientSocket->read(length));

		if (rxlen.indexIn(text) > -1) {
			QString command = rxlen.cap(1);
			double speed = rxlen.cap(2).toDouble();
			double changeInAngle = rxlen.cap(3).toDouble();
			
			std::cout << "Command: " << command.toStdString();
            m_RobotThread.setCommand(command);

			if(command == "SetSpeed") {
				m_RobotThread.SetSpeed(speed, changeInAngle);
				std::cout << "\tX m/sec: " << speed << "\t radians/sec: " << 
changeInAngle;
			}
			std::cout << std::endl;
		}
	}
}
