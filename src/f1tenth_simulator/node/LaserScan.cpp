// Node that subscribes to LaserScan Topic

#include <ros/ros.h>

// For printing messages
#include <ros/console.h>

// Will subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>

// for printing
#include <iostream>

//#include "std_msgs/Float32.h"

class LaserScanSub {
private:
	// Start Node
	ros::NodeHandle n;

	// Listen for Laser messages
	ros::Subscriber LaserScan_sub;

	// Publishers
	ros::Publisher nearest_pub;
	ros::Publisher furthest_pub;

	
public:
	LaserScanSub(){
		//Initialise Node Handle
		n = ros::NodeHandle("~");

		ROS_INFO_STREAM("LaserScan Subscriber Created");

		// get topic names
		std::string laser_scan_topic, near_topic, far_topic;
		n.getParam("scan_topic", laser_scan_topic);
		n.getParam("nearest_point", near_topic);
		n.getParam("furthest_point", far_topic);


		// Create a subscriber to listen to the laser scan messages
		LaserScan_sub = n.subscribe(laser_scan_topic, 1, &LaserScanSub::LaserCallBack, this);

		// Create a publisher for nearest/furthest points
		nearest_pub = n.advertise<float>(near_topic, 1);
		//furthest_pub = n.advertise<float>(far_topic, 1);

	}

	void LaserCallBack(const sensor_msgs::LaserScan & msg){
		//int i = msg.ranges.size();
		//ROS_INFO("Array size: %d", i);
		
		// Define closest and furthest variables
		// Set to first value in array
		float closest = msg.ranges[0];	
		float farthest = msg.ranges[0];	

		//Calculate nearest and furthest points
		for (unsigned int i = 0; i < msg.ranges.size(); i++) {

			if(msg.ranges[i] > farthest  && msg.ranges[i] <= msg.range_max) {

				farthest = msg.ranges[i];

			} else if(msg.ranges[i] < closest  && msg.ranges[i] >= msg.range_min) {

				closest = msg.ranges[i];	
			}
		} 
		ROS_INFO("Closest: %f, Furthest: %f", closest, farthest);
		//nearest_pub.publish(closest);
	}
	
};

int main(int argc, char ** argv){
	ros::init(argc, argv, "LaserScan");
	LaserScanSub ls;
	ros::spin();
	return 0;
	
}


