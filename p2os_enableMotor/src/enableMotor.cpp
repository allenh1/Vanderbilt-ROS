#include "ros/ros.h"
#include "p2os_driver/MotorState.h"

int main( int argc , char** argv )
{
	ros::init( argc , argv , "enableMotor");
	ros::NodeHandle nodeHandle;

 	ros::Publisher _publisher = nodeHandle.advertise<p2os_driver::MotorState>( "cmd_motor_state" , 10 );
	ros::Rate _loop( 10 );
	p2os_driver::MotorState enableFlag;

	enableFlag.state = 1;
	int count = 0;
	while( ros::ok() && ( count < 10 ) )
	{
		
		_publisher.publish( enableFlag );
		_loop.sleep();
		++count;
	}


	return 0;
}
