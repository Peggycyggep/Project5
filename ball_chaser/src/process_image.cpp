#include "ros/ros.h"
#include "ball_chaser/DriveToTarget.h"
#include <sensor_msgs/Image.h>

// Define a global client that can request services
ros::ServiceClient client;
float last_angular;
float last_linear;
float ball_found;

// This function calls the command_robot service to drive the robot in the specified direction
void drive_robot(float lin_x, float ang_z)
{
    // TODO: Request a service and pass the velocities to it to drive the robot
    ball_chaser::DriveToTarget srv;
    srv.request.linear_x = lin_x;
    srv.request.angular_z = ang_z;
  
    if(!client.call(srv))
      ROS_ERROR("Failed to call service DriveToTarget");
}

// This callback function continuously executes and reads the image data
void process_image_callback(const sensor_msgs::Image img)
{
    int max_step, max_row;
    int obj_row_start, obj_row_end, obj_step_start, obj_step_end;
    int pos_row, pos_step, pos;
    int white_pixel = 255;
    float angular=0.0;		//0 center, +val move to left, -val move to right
    float linear = 0.0;		//0 no move, -val move back, +val move forward

    // TODO: Loop through each pixel in the image and check if there's a bright white one
  max_row = img.height;
  max_step = img.step;
  obj_row_start = -1;
  obj_row_end = -1;
  obj_step_start = -1;
  obj_step_end = -1;
  for(pos_row=0, pos=0; pos_row<max_row; pos_row++)
  {
    for(pos_step=0; pos_step<max_step; pos_step+=3)
    {
      pos = pos_row*max_step + pos_step;
      if( (img.data[pos]==white_pixel)&&(img.data[pos+1]==white_pixel)&&(img.data[pos+2]==white_pixel) )
      {
        if(obj_row_start==-1)
        {
          obj_row_start = pos_row;
          obj_row_end = pos_row;
        }
        else
          obj_row_end = pos_row;
        
        if(obj_step_start==-1)
        {
          obj_step_start = pos_step;
          obj_step_end = pos_step;
        }
        else if(obj_step_start>pos_step)
          obj_step_start = pos_step;
        else if(obj_step_end<pos_step)
          obj_step_end = pos_step;        
      }
    }
  }
  
    // Then, identify if this pixel falls in the left, mid, or right side of the image      
  if( obj_step_start>-1 )
  {
    float obj_center = (float)(obj_step_end-obj_step_start)/2.0 + obj_step_start;
    float step_center = (float)(max_step)/2.0;
    float deg = (obj_center / (float)max_step - 0.5) * -2;

    ball_found = 1;
    angular = deg;
    linear = 0.2;
  }
  else if(ball_found==0)
  {
    linear = 0.1;
    angular = 1;		//turn around (rad/sec)
  }
  // Request a stop when there's no white ball seen by the camera
  else
  {
    linear = 0;
    angular = 0;
  }
  
  // Depending on the white ball position, call the drive_bot function and pass velocities to it
  if( (last_angular!=angular)||(last_linear!=linear) )
  {
    last_angular = angular;
    last_linear = linear;
  	drive_robot(linear, angular);
  }
}

int main(int argc, char** argv)
{
    // Initialize the process_image node and create a handle to it
    ros::init(argc, argv, "process_image");
    ros::NodeHandle n;
    
    ball_found = 0;

    // Define a client service capable of requesting services from command_robot
    client = n.serviceClient<ball_chaser::DriveToTarget>("/ball_chaser/command_robot");

    // Subscribe to /camera/rgb/image_raw topic to read the image data inside the process_image_callback function
    ros::Subscriber sub1 = n.subscribe("/camera/rgb/image_raw", 10, process_image_callback);

    // Handle ROS communication events
    ros::spin();

    return 0;
}
