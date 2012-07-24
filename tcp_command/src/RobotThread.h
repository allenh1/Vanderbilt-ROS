#ifndef ___ROBOTTHREAD_H___
#define ___ROBOTTHREAD_H___

#include <QThread>
#include <ros/ros.h>
#include <ros/network.h>
#include <std_msgs/String.h>

class RobotThread : public QThread {
	Q_OBJECT
public:
    RobotThread(int argc, char** argv);
    virtual ~RobotThread();
	void run();
    bool init();
	void SetSpeed(double speed, double angle);
    void setCommand(QString cmd);

	//get speed in forward/backward direction
	double getXSpeed();

	//get x,y position and angular pose	
	double getXpos();
	double getYpos();
	double getApos();

	//built in goto command to a goal position x
    //void goTo(player_pose2d_t x);



	//get obstacle distance in front of robot using laser sensor
	double ForwardObDist(int mode = 0);
	

	void EndControl();

	void Read();

private:
	/** meters per second */
	double m_Speed;

	/** degrees per second */
	double m_Angle;

    QString command;
	/** the robot is not moving */
	bool m_IsStopped;

	/** continue driving the robot */
	bool m_Continue;

	int m_port;
	
    int init_argc;
    char** init_argv;

    ros::Publisher cmd_publisher;
	/** a position 2d proxy */

	
};
#endif
