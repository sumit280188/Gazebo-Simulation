#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // Request a service and pass the velocities to it to drive the robot
    
    ROS_INFO_STREAM("Giving velocity to the robot");

    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;

      if (!client.call(srv))
        ROS_ERROR("Failed to call service DriveToTarget");

}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int white_pixel = 255;
    bool ball_detected = false;

    // Loop through each pixel in the image and check if there's a bright white one
    for (int i = 0; i < (img.height * img.step); i+=3) {
        if (img.data[i] == white_pixel && img.data[i + 1] == white_pixel && img.data[i + 2] == white_pixel) {
            ball_detected = true;
            int column = i % img.step; // Calculate the column of the pixel
            // Depending on the position of the white pixel, call the drive_robot function with appropriate velocities
            if (column < img.step / 3) {
                // White pixel in the left third of the image
                drive_robot(0.5, -0.5); // Turn left
            } else if (column < 2 * img.step / 3) {
                // White pixel in the middle third of the image
                drive_robot(0.5, 0.0); // Move forward
            } else {
                // White pixel in the right third of the image
                drive_robot(0.5, 0.5); // Turn right
            }
            break; // Break out of the loop after detecting the first white pixel
        }
    }

    // If no white pixel is detected, stop the robot
    if (!ball_detected) {
        drive_robot(0.0, 0.0); // Stop
    }
}


int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
