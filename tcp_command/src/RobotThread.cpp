#include "RobotThread.h"

RobotThread::RobotThread(QObject* pParent)
:	QThread(pParent),
	m_Speed(0),
	m_Angle(0),
	m_IsStopped(true),
	m_Continue(true),
	m_Robot("127.0.0.1"),	// Connect to robot using a default standard port
	m_Position(&m_Robot, 0) {	// Create a position 2d proxy

	// Enable the motors
	m_Position.SetMotorEnable(true);
}

void RobotThread::run() {
		
	while(m_Continue) {
		m_Robot.Read();
		
		if((m_IsStopped && (m_Speed != 0 && m_Angle !=0)) || !m_IsStopped) {
			m_Position.SetSpeed(m_Speed, m_Angle);
		}

		// Update IsStopped
		m_IsStopped = m_Speed == 0 && m_Angle == 0;
	}

	m_Position.SetSpeed(0, 0);
}

void RobotThread::SetSpeed(double speed, double angle) {
	m_Speed = speed;
	m_Angle = angle;
}

double RobotThread::getForwardSpeed()
{
	return m_Speed;
}

double RobotThread::getTurnSpeed()
{
	return m_Angle;
}

void RobotThread::EndControl() {
	m_Continue = false;
}
