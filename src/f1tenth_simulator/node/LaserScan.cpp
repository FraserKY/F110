// Node that subscribes to LaserScan Topic

#include <ros/ros.h>

// For printing messages
#include <ros/console.h>

// Will subscribe to a topic with this message type
#include <sensor_msgs/LaserScan.h>

// for printing
#include <iostream>

class LaserScanSub {
private:
	// Start Node
	ros::NodeHandle n;

	// Listen for Laser messages
	ros::Subscriber LaserScan_sub;

public:
	LaserScanSub(){
		//Initialise Node Handle
		n = ros::NodeHandle("~");

		ROS_INFO_STREAM("LaserScan Subscriber Created");

		//std::cout << "Running";

		// get topic names
		std::string laser_scan_topic;
		n.getParam("scan_topic", laser_scan_topic);
		
		// Create a subscriber to listen to the laser scan messages
		LaserScan_sub = n.subscribe(laser_scan_topic, 1, &LaserScanSub::LaserCallBack, this);
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
	}
	
};

int main(int argc, char ** argv){
	ros::init(argc, argv, "LaserScan");
	LaserScanSub ls;
	ros::spin();
	return 0;
	
}


