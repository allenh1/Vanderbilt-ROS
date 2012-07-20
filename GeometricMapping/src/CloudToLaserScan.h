#ifndef CLOUD_TO_LASER_H
#define CLOUD_TO_LASER_H
#include<pcl_ros/point_cloud.h>
#include<pcl/point_types.h>
#include<pcl/io/pcd_io.h>
#include<Eigen/StdVector>
#include <string>
#include<iostream>
#include "ros/ros.h"
#include "sensor_msgs/LaserScan.h"
const std::string frame_id = "/filtered";

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;

class Cloud_to_Laser
{
	public:
		Cloud_to_Laser(sensor_msgs::LaserScan, PointCloud);
		sensor_msgs::LaserScan cloudToLaser();

	private:
		sensor_msgs::LaserScan toPublish;
		unsigned int cloudSize;
		PointCloud cloud;
		unsigned int seq;
		float angle_min; 
		float angle_max;
		float angle_increment;
		float time_increment;
		float scan_time;
		float range_min;
		float range_max;
		float *ranges;
		float *intensities;
};

Cloud_to_Laser::Cloud_to_Laser(sensor_msgs::LaserScan toSimulate, PointCloud cloudin)
	:	angle_min(toSimulate.angle_min), 
		angle_max(toSimulate.angle_max), 
		angle_increment(toSimulate.angle_increment)
{
	scan_time = toSimulate.scan_time;
	time_increment = toSimulate.time_increment;
	range_min = toSimulate.range_min;
	range_max = toSimulate.range_max;
	intensities = new float[cloudin.size()];

	for (unsigned int x = 0;  x < cloudin.size(); x++)
		intensities[x] = toSimulate.intensities[x];

	ranges = new float[cloudin.size()];
	seq = toSimulate.header.seq;
	cloudSize = cloudin.points.size();
}//set up the the scan data. 

sensor_msgs::LaserScan Cloud_to_Laser::cloudToLaser()
{
	sensor_msgs::LaserScan toPublish;
	for (unsigned int iter = 0; iter < cloudSize; iter++)
	{
		double currentAngle = angle_min + angle_increment * iter; //get the current angle
		ranges[iter] = cloud.points.at(iter).x / cos(currentAngle);
	}//convert the points to range values. 
	
	toPublish.angle_min = angle_min;
	toPublish.angle_max = angle_max;
	toPublish.angle_increment = angle_increment;
	toPublish.time_increment = time_increment;
	toPublish.scan_time = scan_time;
	toPublish.range_min = range_min;
	toPublish.range_max = range_max;
	toPublish.intensities = intensities;
	toPublish.ranges = ranges;
	toPublish.header.seq = seq;
	toPublish.header.stamp = ros::Time::now();
	toPublish.header.frame_id = frame_id;
	delete[] ranges;
	return toPublish;
}//returns a sensor_msgs::LaserScan
#endif
