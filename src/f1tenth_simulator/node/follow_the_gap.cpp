// This node implements an algorithm to find the largest gap seen by the laser scan, and directs the vehicle towards it.

#include <ros/ros.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

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

    void LidarCallBack(const sensor_msgs::LaserScan & msg){
        // preprocess lidar
        // Create pointer for lidar processed:

        double lidar[1080] = {};

        for (int x = 0; x < 1080; x++){
            lidar[x] = msg.ranges[x];
            // Successfully copies array from msg.
            // ROS_INFO_STREAM("Lidar [" << x << "]: = " << lidar[x]);
            // ROS_INFO_STREAM("Callback [" << x << "]: = " << msg.ranges[x]);

        }

        //ROS_INFO_STREAM("Vector: " << scan_ranges);

        PreProcessArray(lidar);

        //ROS_INFO_STREAM("Lidar Array Change Test: " << lidar[b]);

        // Calculate array size
        int array_size = sizeof(lidar) / sizeof(lidar[0]);

        // Create a pointer to access return values from LCNZG function
        int *p;

        // Find the largest consecutive non-zero gap
        p = LargestConsecutiveNonZeroGap(lidar, array_size);


    }

    void PreProcessArray(double lidar[1080]) {

        double min_val = 100;
        int min_index;

        // Find the smallest value in the array
        for (int x = 0; x < 1080; x++) {

            if (lidar[x] < min_val) {

                min_val = lidar[x];

                min_index = x;

            }

        }

        // Print the smallest value in an array
        //ROS_INFO_STREAM("Smallest value: " << min_val);

        //ROS_INFO_STREAM("Index of smallest value: " << min_index);

        // Make smallest value and surrounding values 0
        for (int x = -3; x < 3; x++){

            lidar[min_index + x] = 0.0;

            //ROS_INFO_STREAM("Lidar Index value " << min_index << " " << lidar[min_index + x]);

        }

        // Set all values not within a 120 degree view forwards centred around the x axis to 0
        for (int x = 0; x < 1080; x++) {
            if (x < 360 or x > 720){
                lidar[x] = 0.0;
            }
        }

        //return min_index;
    }


    int * LargestConsecutiveNonZeroGap(const double lidar[], int array_size){

        int start_index, end_index, length;
        int start_index_longest, end_index_longest, length_longest = 1;

        // Loop through array to find largest gap

        for(int x = 0; x < array_size; x++){

            // Set the start index to the current index
            start_index = x;
            // Set the length to zero
            length = 0;

            while(lidar[x] > 0.0 and x < array_size){
                // While the value stored at that index is greater than 0, increment length
                length++;
                x++;

            }

            end_index = x;
            x = end_index;

            if(length > length_longest){

                // If the current run is longer than any previously found run, updated the start index, end index and length
                start_index_longest = start_index;
                end_index_longest = end_index - 1;
                length_longest = length;

            }

        }

        //cout << "The longest length of consecutive non-zero values was: " << length_longest << endl;
        //cout << "The start index was: " << start_index_longest << endl;
        //cout << "The end index was: " << end_index_longest << endl;

        //ReturnMiddleOfGap(r);
        // Find the middle of the gap
        int middle_index = (start_index + end_index) / 2;

        //cout << "The middle index is: " << middle_index << endl;

        static int r[4] = {middle_index, start_index_longest, end_index_longest, length_longest};

        return r;


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
