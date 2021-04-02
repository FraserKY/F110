#include <ros/ros.h>

//Message Types
#include <ackermann_msgs/AckermannDriveStamped.h>
#include <ackermann_msgs/AckermannDrive.h>
#include <sensor_msgs/LaserScan.h>

// Other Libraries
#include <math.h>

class Wall_Follower{
private:
    ros::NodeHandle n;

    // Create Subscribers
    ros::Subscriber LaserScan;

    ros::Publisher DriveMessage:

    // TODO: Precompute cos angle of LS data

public:
    Wall_Follower(){
        // Start Node
        n = ros::NodeHandle("~");

        // TODO: Set up subscribers and publishers




        // Define any const variables

        const int Angle_A = 60;
        const double Angle_A_Rad = Angle_A * (M_PI / 180);

        // Distance from wall setpoint
        const double SetPoint = 1;


    }

    // TODO: Create callback function for Laser Scan info

    LS_callback(const sensor_msgs::LaserScan::ConstPtr &scan_msg){

        // Obtain LS distance at exactly East of Car (B), and 60 degrees anticlockwise of this point (A)

        double B = scan_msg->Ranges[?];
        double A = scan_msg->Ranges[?];

        // Calculate Alpha (Alpha), the angle between exactly east of the car, and the shortest distance to the wall

        double alpha;

        alpha = atan(   (A * cos(Angle_A) - B)   /    (A * sin(Angle_A)) );   // Result in radians

        // Calculate the distance of the LIDAR sensor from the wall (Dt)

        double Dt = B * cos(alpha);

        // Calculate Dt at next time step, Distance covered in next time step (L),
        // TODO: Calculate distance at next time step based on speed

        // Current Speed * Time Step
        double L =  1 * 1 ;

        double Dt_1 = Dt + L * sin(alpha);

        // Calculate Error, Set Point (SetPoint) - Dt_1

        double Error = SetPoint - Dt_1;

        // Call PID Function
        // TODO: Write PID Function

        // Call Publish Function
        // TODO: Write Publish function

    }


};



int main(int argc, char ** argv) {
    ros::init(argc, argv, "wall_follower");
    Safety wall_follower;
    ros::spin();
    return 0;
}