#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <gnc_functions.hpp>
#include <sensor_msgs/PointCloud.h>

void vision_cb(const sensor_msgs::PointCloud::ConstPtr& msg)
{
	sensor_msgs::PointCloud current_3D_scan;
	current_3D_scan = *msg;

	
}


int main(int argc, char **argv)
{
	//initialize ros 
	ros::init(argc, argv, "avoidance_node");
	ros::NodeHandle n;

	ros::Subscriber collision_sub = n.subscribe<sensor_msgs::PointCloud>("/camera/depth/color/points", 1, vision_cb);

	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

	// wait for FCU connection
	wait4connect();

	//wait for user to switch to mode GUIDED
	wait4start();

	//create local reference frame 
	initialize_local_frame();

	//request takeoff
	takeoff(5);

	set_destination(15,15,10,0);

	ros::Rate rate(2.0);
	int counter = 0;
	while(ros::ok())
	{
		
		ros::spinOnce();
		rate.sleep();
	}

	return 0;
}