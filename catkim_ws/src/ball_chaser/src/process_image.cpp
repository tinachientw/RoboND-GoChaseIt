#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ROS_INFO_STREAM("Driving the robot to the target.");
    
    // Request service with velocities
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
    
    // Call the DriveToTarget service and pass the requested velocities
    if (!client.call(srv)) {
        ROS_ERROR("Failed to call service DriveToTarget.");
    }
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{

    int white_pixel = 255;
    float lin_x = 0.0;
    float ang_z = 0.0;
    float ball_position_sum = 0.0;
    int ball_pixel = 0;

    // TODO: Loop through each pixel in the image and check if there's a bright white one
    // Then, identify if this pixel falls in the left, mid, or right side of the image
    // Depending on the white ball position, call the drive_bot function and pass velocities to it
    // Request a stop when there's no white ball seen by the camera
    
    //go through the pixel of img to identify ball pixel and position
    for (int i = 0; i < img.height ; i++) {
        for (int j = 0; j < img.step; j+=3) {
            if (img.data[i * img.step + j] == white_pixel   && //R 
		img.data[i * img.step + j+1] == white_pixel && //G
                img.data[i * img.step + j+2] == white_pixel) { //B 
		ball_position_sum += j;
                ball_pixel++;
            }
        }
    }

    // If there is no pixel or it's too close to the ball, then tell the car to stop
    if (ball_pixel == 0 || 
        ball_pixel > 0.15 * (img.height * img.step) ) {
        lin_x = 0.0;
        ang_z = 0.0;

    }
    else {
        lin_x = 0.1;
        ang_z = -0.5 * (ball_position_sum/ball_pixel - img.step/2.0) / (img.step/2.0) ;
    }

    // Send request to service
    drive_robot(lin_x, ang_z);
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
