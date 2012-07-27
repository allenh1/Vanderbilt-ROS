#include <qt4/QtNetwork/QtNetwork>

#include "Server.h"
namespace server {

Server::Server(int argc, char** argv, QObject* pParent)
    :	QObject(pParent),
      m_RobotThread(argc, argv)
{

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
    m_RobotThread.init();
    m_RobotThread.start();
}

void Server::NewClientConnection() {
	QTcpSocket * pClientSocket = m_pTcpServer->nextPendingConnection();
	if(pClientSocket) {
		connect(pClientSocket, SIGNAL(disconnected()), pClientSocket, SLOT(deleteLater()));
		connect(pClientSocket, SIGNAL(readyRead()), SLOT(NewClientCommand()));
	}
}

void Server::writeData()
{
    //QTcpSocket *pClientSocket = qobject_cast<QTcpSocket*>(sender());
    //Q_UNUSED(pClientSocket);
    //const QRegExp rxlen("^(\\w+)\\s+(-*\\d*\\.?\\d*)\\s+(-*\\d*\\.?\\d*)$");
    //QString text(pClientSocket->read(length));
	
    //pClientSocket->write(m_RobotThread.getForwardSpeed());
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
            QString l_speed; QString a_speed;
            double changeInAngle = rxlen.cap(3).toDouble();
            l_speed.setNum(speed); a_speed.setNum(changeInAngle);

            QString toRos = command + ", m/sec: " + l_speed + ", rad/sec: " + a_speed;
            std::cout << "Command: " << command.toStdString();
            m_RobotThread.setCommand(toRos);

            if (command == "SetSpeed")
            {
                m_RobotThread.SetSpeed(speed, changeInAngle);
                std::cout << "\tX m/sec: " << speed << "\t radians/sec: " <<
changeInAngle;
            }

            else if (command == "getPosition")
            {
                std::stringstream stm;
                stm << "("; stm << m_RobotThread.getXPos();
                stm << ", "; stm << m_RobotThread.getYPos();
                stm << ", "; stm << m_RobotThread.getAPos();
                stm << ")"; //format the string in a stringstream.

                std::string pos = stm.str();

                pClientSocket->write(pos.c_str());
            }//if the command is a get position.

            else if (command == "getXSpeed")
            {
                std::stringstream stm;
                stm << m_RobotThread.getXSpeed();
                pClientSocket->write(stm.str().c_str());
            }//returns the linear speed of the robot to tcp.

            else if (command == "getASpeed")
            {
                std::stringstream stm;
                stm << m_RobotThread.getASpeed();
                pClientSocket->write(stm.str().c_str());
            }//returns the angular speed of the robot to tcp.

            else if (command == "setPosition") {
                double xPos = speed;
                double yPos = changeInAngle;

                QList<double> pose;
                pose.push_back(xPos);
                pose.push_back(yPos);
                m_RobotThread.setPose(pose);
            }//end else if

            else if (command == "setGoal")
            {
                double xPos = speed, yPos = changeInAngle;

                geometry_msgs::Point goal;
                goal.x = xPos;
                goal.y = yPos;
                goal.z = PI/2;
                m_RobotThread.goToXYZ(goal);
            }//end else if

            std::cout << std::endl;
        }
    }
}

/*void Server::NewClientCommand() {
    //const QRegExp rxlen("^(\\w+)\\s+(-*\\d*\\.?\\d*)\\s+(-*\\d*\\.?\\d*)$");
    const QRegExp rxlen("^(\\w+)\\s+(-*\\d*\\.?\\d*)\\s+(-*\\d*\\.?\\d*)\\s+(-*\\d*\\.?\\d*)$");
    int length;

    QTcpSocket * pClientSocket = qobject_cast<QTcpSocket *>(sender());

    while(pClientSocket->bytesAvailable() > 0) {
        length = static_cast<int>(pClientSocket->read(1).at(0));	// Read the command size.
        QString text(pClientSocket->read(length));

        if (rxlen.indexIn(text) > -1) {
            QString command = rxlen.cap(1);
            double speed = rxlen.cap(2).toDouble();
            double changeInAngle = rxlen.cap(3).toDouble();
            double par4 = rxlen.cap(4).toDouble();

            m_RobotThread.setCommand(command);

            else if(command == "getGoalDirObDist") {
                //std::cout << "before read is good. " << std::endl;
                //m_RobotThread.Read();
                //std::cout << "after read is good. " << std::endl;
                std::string pos = stringify(m_RobotThread.goalDirObDist());
                pClientSocket->write(pos.c_str());
            }
            else if(command == "goTo") {
                double xPos = speed,yPos = changeInAngle;

                player_pose2d_t pos = {xPos,yPos,0}, vel = {par4,0,0};
                std::cout << "target pos: " << pos << " vel: " << vel << std::endl;
                m_RobotThread.goTo(pos,vel);
            }//end else if
            else if(command == "go") {
                double xPos = speed,yPos = changeInAngle,yaw = par4;

                player_pose2d_t pos = {xPos,yPos,yaw};
                std::cout << "target pos: " << pos << std::endl;
                m_RobotThread.goTo(pos);
            }//end else if
            else if(command == "getForwardObDist") {
                int mode = speed;
                std::string dist = stringify(m_RobotThread.ForwardObDist(mode));
                pClientSocket->write(dist.c_str());
            }//end else if
            else if(command == "setMotor") {
                double mode = speed; bool enable = true;
                if (mode==0)
                    enable = false;
                m_RobotThread.setMotor(enable);
            }//end else if
        }//end if
    }//end while
}//callback for tcp messages.*/
}//namespace
