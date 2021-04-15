// This node implements an algorithm to find the largest gap seen by the laser scan, and directs the vehicle towards it.

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>

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

    void LidarCallBack(const sensor_msgs::LaserScan &msg){
        // preproccess lidar
        // Create pointer for lidar proccessed:
        double *lidar_proc;

        lidar_proc = preprocess_lidar(msg->ranges[]);

    }

    double* preprocess_lidar(double ranges[]){
        //
        double min_val = 100;
        int min_index;
        int x;

        // Find the smallest value in the array
        for (x = 0; x >= 1080; x++){

            if (ranges[x] < min_val){

                min_val = ranges[x];

                min_index = x;

            }

        }


        std::cout << "Smallest value: " << min_val << std::endl;

        std::cout << "Index of smallest value: " << min_index << std::endl;

        // Make smallest value and surrounding values 0

        for (x = -3; x >= 3; x++){

            ranges[min_index + x] = 0.0;

        }

        return ranges;


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
