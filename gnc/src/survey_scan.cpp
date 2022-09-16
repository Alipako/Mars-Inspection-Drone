#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <darknet_ros_msgs/BoundingBoxes.h>
#include <darknet_ros_msgs/BoundingBox.h>
#include <gnc_functions.hpp>
#include <cmath>



using namespace std;

int mode_g = 0; // 0 cautam crater 1 am gasit
float length = 0;
float width = 0;
float aria = 0;
int altitude = 10; 

void detection_cb(const darknet_ros_msgs::BoundingBoxes::ConstPtr& msg)
{
	for( int i=0; i < msg->bounding_boxes.size(); i++)
	{
		ROS_INFO("%s detected", msg->bounding_boxes[i].Class.c_str());	
		

		ROS_INFO("lenght = %f[m]", (msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin)*(4*tan(1.272345/2)*altitude/640));
		ROS_INFO("width = %f[m]", (msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin)*(4*tan(1.007055/2)*altitude/480));



		if(msg->bounding_boxes[i].Class == "cave")
		{
			mode_g = 1;


			//ROS_INFO("lenght = %f[m]", (msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin)*(2*tan(1.272345/2)*altitude/640));
			//ROS_INFO("width = %f[m]", (msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin)*(2*tan(1.007055/2)*altitude/480));
			
			aria = (msg->bounding_boxes[i].xmax - msg->bounding_boxes[i].xmin)*(4*tan(1.272345/2)*2*altitude/640)*
			(msg->bounding_boxes[i].ymax - msg->bounding_boxes[i].ymin)*(4*tan(1.007055/2)*2*altitude/480);

			ROS_INFO("Crater Aria = %f[m]", aria);
			
		}

	}	

}


int main(int argc, char **argv)
{
	ros::init(argc, argv, "detection_sub");
	ros::NodeHandle n;
	ros::Subscriber sub = n.subscribe("/darknet_ros/bounding_boxes", 1, detection_cb);



	//initialize control publisher/subscribers
	init_publisher_subscriber(n);

	// asteptam conectiunea cu FCU
	wait4connect();

	//Se va cere modul confirmare de zbor prin setarea modului GUIDED
	wait4start();

	//se va genera un sistem de referinta local
	initialize_local_frame();

	//request takeoff
	takeoff(2);

	//specificam punctele din traseu 
	std::vector<gnc_api_waypoint> waypointList;
	gnc_api_waypoint nextWayPoint;

	nextWayPoint.z = altitude;

	float range = 23; // lungime
	float spacing = 11; // latimi micute
	int row;
	for(int i=0; i<15; i++)
	{
		row = i*2; 
		nextWayPoint.x = 0;
		nextWayPoint.y = row*spacing;
		nextWayPoint.z = altitude;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
		
		nextWayPoint.x = range;
		nextWayPoint.y = row*spacing;
		nextWayPoint.z = altitude;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
		
		nextWayPoint.x = range;
		nextWayPoint.y = (row+1)*spacing;
		nextWayPoint.z = altitude;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);

		nextWayPoint.x = 0;
		nextWayPoint.y = (row+1)*spacing;
		nextWayPoint.z = altitude;
		nextWayPoint.psi = 0;
		waypointList.push_back(nextWayPoint);	
	}




	ros::Rate rate(2.0);// 2 hz
	int counter = 0;

	while(ros::ok())
	{	
		if(mode_g == 0 )
		{
			ros::spinOnce();
			rate.sleep();
			if(check_waypoint_reached(1.2) == 1) // tipa se pune 0.3 a fi toleranta (adica cat de aproape drona trebuie sa fie de punct ca el sa fie considerat atins)
			{
				if(counter < waypointList.size())
				{
					set_destination(waypointList[counter].x, waypointList[counter].y, waypointList[counter].z, waypointList[counter].psi);
					counter++;
				}else{
					//drone did not find the lava tube
					set_destination(waypointList[0].x, waypointList[0].x, waypointList[0].z, waypointList[0].psi);
					land();
				}
			}	
		}  
		if(mode_g == 1) 
		{
			rate.sleep();
			land();

		    ROS_INFO("Inspection finished");
		    break;
		} 
	}

	
	return 0;
}