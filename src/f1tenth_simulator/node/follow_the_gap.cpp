// This node implements an algorithm to find the largest gap seen by the laser scan, and directs the vehicle towards it.

#include <ros/ros.h>
#include <math.h>

#include <ackermann_msgs/AckermannDriveStamped.h>
#include <sensor_msgs/LaserScan.h>

using namespace std;

class follow_the_gap{
private:
    // Define node handle
    ros::NodeHandle n;

    // Subscriber for Laser Scan Data
    ros::Subscriber LaserScanSub;

    // Publisher for steering input and speed
    ros::Publisher DriveMessagePub;

public:
    follow_the_gap(){
        //Initialise Node Handle
        n = ros::NodeHandle("~");

        std::string laser_scan_topic, drive_topic;
        n.getParam("scan_topic", laser_scan_topic);
        n.getParam("follow_the_gap_topic", drive_topic);

        // Create a subscriber to listen to the laser scan messages
        LaserScanSub = n.subscribe(laser_scan_topic, 1, &follow_the_gap::LidarCallBack, this);

        // Create a publisher to send drive commands
        DriveMessagePub = n.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
    }

    void LidarCallBack(const sensor_msgs::LaserScan & msg){
        // preprocess lidar
        // Create pointer for lidar processed:

        double lidar[1080] = {};
        double steering_angle;
        int steering_dir_index;

        for (int x = 0; x < 1080; x++){
            lidar[x] = msg.ranges[x];
            // Successfully copies array from msg.
            // ROS_INFO_STREAM("Lidar [" << x << "]: = " << lidar[x]);
            // ROS_INFO_STREAM("Callback [" << x << "]: = " << msg.ranges[x]);

        }

        PreProcessArray(lidar);

        // Calculate array size
        int array_size = sizeof(lidar) / sizeof(lidar[0]);


        // Find the largest consecutive non-zero gap
        steering_dir_index = LargestConsecutiveNonZeroGap(lidar, array_size);

        //ROS_INFO_STREAM("Steering Goal Index: " << steering_dir_index);

        steering_angle = DetermineSteeringAngle(steering_dir_index);

        // Create nav message
        ackermann_msgs::AckermannDriveStamped drive_st_msg;
        ackermann_msgs::AckermannDrive drive_msg;

        // TODO: Function to determine speed based on steering angle

        drive_msg.steering_angle = steering_angle;
        drive_msg.speed = DetSpeed(steering_angle);

       // ROS_INFO_STREAM("Speed: " << drive_msg.speed);

        // Set Ackerman Message to a Stamped Ackermann Message
        drive_st_msg.drive = drive_msg;

        // publish AckermannDriveStamped message to drive topic
        DriveMessagePub.publish(drive_st_msg);
    }

    void PreProcessArray(double lidar[1080]) {

        double min_val = 100;
        int min_index;

        // Find the smallest value in the array within 120 degree front view
        for (int x = 270; x < 810; x++) {

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
            if (x < 270 or x > 810){
                lidar[x] = 0.0;
            }
        }

        //cout << "Lidar index 800: " << lidar[800] << endl;

        //cout << "Lidar index 120: " << lidar[120] << endl;

        //return min_index;
    }


    int LargestConsecutiveNonZeroGap(const double lidar[], int array_size){

        int start_index, end_index, length, middle_index;
        int start_index_longest, end_index_longest, length_longest = 1;

        // Loop through array to find largest gap

        for(int x = 0; x < array_size; x++){

            // Set the start index to the current index
            start_index = x;
            // Set the length to zero
            length = 0;

            // TODO: dynamically update threshold?
            while(lidar[x] > 1.0 and x < array_size){
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

        //ROS_INFO_STREAM("The longest length of consecutive non-zero values was: " << length_longest);
        //ROS_INFO_STREAM("The start index was: " << start_index_longest);
        //ROS_INFO_STREAM("The end index was: " << end_index_longest);

        //ReturnMiddleOfGap(r);
        // Find the middle of the gap
        middle_index = (start_index_longest + end_index_longest) / 2;

        //cout << "The middle index is: " << middle_index << endl;

        //static int r[4] = {middle_index, start_index_longest, end_index_longest, length_longest};

        return middle_index;


    }

    double DetermineSteeringAngle(int middle_of_gap_index){

        // This function determines the steering angle based on the index of the middle of the largest gap
        int car_middle_index = 540;
        double steering_angle, angle_increment = 0.33;

        // Each index is covers approx 0.3 degrees
        steering_angle = (middle_of_gap_index - car_middle_index) * angle_increment;

        // Convert to radians
        steering_angle = steering_angle * (M_PI/180);

        // Debugging for steering angle
        ROS_INFO_STREAM("Steering angle: " << steering_angle << " rads");

        return steering_angle;
    }

    double DetSpeed(const double steering_angle_rads){
        // Function determines speed based on steering angle
        if(steering_angle_rads <= 0.05){
            return 4.0;
        }
        else if (steering_angle_rads <= 0.35 and steering_angle_rads > 0.05){
            return 1.0;
        }

        else if(steering_angle_rads > 0.35 and steering_angle_rads <= 0.40){
            return 1.0;
        }
        else if(steering_angle_rads > 0.4){
            return 0.5;
        }
        else{
            return 0;
        }
    }

};

int main(int argc, char ** argv){
    // Initialise Node
    ros::init(argc, argv, "follow_the_gap");
    // Call class function
    follow_the_gap ftg;
    // Receive all callbacks
    //ros::spin();

    ros::Rate loop_rate(10);

    while (ros::ok()) {
        loop_rate.sleep();
        ros::spinOnce();
    }


    return 0;
}
