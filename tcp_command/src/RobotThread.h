#ifndef ___ROBOTTHREAD_H___
#define ___ROBOTTHREAD_H___

#include <QThread>
#include <libplayerc++/playerc++.h>
#include <libplayerc++/playerclient.h>

class RobotThread : public QThread {
	Q_OBJECT
public:
	RobotThread(int port, QObject* pParent = NULL);

	void run();

	void SetSpeed(double speed, double angle);

	//get speed in forward/backward direction
	double getXSpeed();

	//get x,y position and angular pose	
	double getXpos();
	double getYpos();
	double getApos();

	//built in goto command to a goal position x
	void goTo(player_pose2d_t x);



	//get obstacle distance in front of robot using laser sensor
	double ForwardObDist(int mode = 0);
	

	void EndControl();

	void Read();

private:
	/** meters per second */
	double m_Speed;

	/** degrees per second */
	double m_Angle;

	/** the robot is not moving */
	bool m_IsStopped;

	/** continue driving the robot */
	bool m_Continue;

	int m_port;

	PlayerCc::PlayerClient m_Robot;

	player_pose2d_t goal_Pose;
	
	/** a position 2d proxy */
	PlayerCc::Position2dProxy m_Position;
	PlayerCc::Position2dProxy m_Position_1;

	PlayerCc::LaserProxy m_Laser;

	
};
#endif
