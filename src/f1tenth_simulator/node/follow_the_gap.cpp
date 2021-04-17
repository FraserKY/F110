// This node implements an algorithm to find the largest gap seen by the laser scan, and directs the vehicle towards it.

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>
#include <vector>

class follow_the_gap{
private:
    // Define node handle
    ros::NodeHandle n;

    // Subscriber for Laser Scan Data
    ros::Subscriber LaserScan_sub;

    // Publisher for steering input and speed
    ros::Publisher Nav_sub;

public:
    follow_the_gap(){
        //Initialise Node Handle
        n = ros::NodeHandle("~");

        std::string laser_scan_topic;
        n.getParam("scan_topic", laser_scan_topic);

        // Create a subscriber to listen to the laser scan messages
        LaserScan_sub = n.subscribe(laser_scan_topic, 1, &follow_the_gap::LidarCallBack, this);
    }

    void LidarCallBack(sensor_msgs::LaserScan& msg){
        // preprocess lidar
        // Create pointer for lidar processed:
        //double *lidar_proc;

        //double lidar[msg.ranges.size()] = msg.ranges;

        std::vector<float> scan_ranges = convertLSRanges(msg);

        ROS_INFO_STREAM("Vector: " << scan_ranges);

        //preprocess_lidar(msg);

    }

    std::vector<float> convertLSRanges(const sensor_messages::LaserScan::ConstPtr& scan_msg){

        return std::vector<float>(scan_msg.ranges.begin(), scan_msg.ranges.end());

    }


    void preprocess_lidar(std::vector<float> lidar){
        //
        double min_val = 100;
        int min_index;

        //cast

        // Find the smallest value in the array
        for (int x = 0; x >= 1080; x++){

            if (lidar[x] < min_val){

                min_val = lidar[x];

                min_index = x;

            }

        }


        ROS_INFO_STREAM("Smallest value: " << min_val);

        ROS_INFO_STREAM("Index of smallest value: " << min_index);

        // Make smallest value and surrounding values 0

       /* for (int x = -3; x >= 3; x++){

            lidar[min_index + x] = 0.0;

        }*/

    }

};

int main(int argc, char ** argv){
    // Initialise Node
    ros::init(argc, argv, "follow_the_gap");
    // Call class function
    follow_the_gap ftg;
    // Receive all callbacks
    ros::spin();

    return 0;
}
