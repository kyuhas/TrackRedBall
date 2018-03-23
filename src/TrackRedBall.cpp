#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h>
#include "std_msgs/String.h"

static const std::string OPENCV_WINDOW = "Image window";
const double EMPTY = -1;
const double MID_X_LOW = 310;
const double MID_X_HIGH = 330;
const double one_meter_distance_low = 980; 
const double one_meter_distance_high = 1020;

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Subscriber image_depth_sub_;
  image_transport::Publisher image_pub_;
  cv_bridge::CvImagePtr cv_depth_ptr;
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscribe to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, 
      &ImageConverter::imageCb, this);
    image_depth_sub_ = it_.subscribe("/camera/depth/image_raw", 10, 
       &ImageConverter::setDepth, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 10);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void setDepth(const sensor_msgs::ImageConstPtr& msg) 
  {
      cv_depth_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat hsv_image;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // Convert priginal image to HSV
      cv::cvtColor(cv_ptr->image, hsv_image, cv::COLOR_BGR2HSV);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }
    
    //Publisher of command velocities
    ros::Publisher vel_pub = nh_.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 1);
    //new Twist message (starts as all zeros)
    geometry_msgs::Twist vel; 

    // REMOVE ALL PIXELS BUT RED/ORANGE ONES
    //use thresholding to keep only red and orange pixels 
    //to find thresholded color ranges, I used this website: http://www.tydac.ch/color/
    cv::Mat red_pixels_1;
    cv::Mat red_pixels_2;
    cv::inRange(hsv_image, cv::Scalar(0, 100, 100), cv::Scalar(20, 255, 255), red_pixels_1);
    cv::inRange(hsv_image, cv::Scalar(160, 100, 100), cv::Scalar(170, 255, 255), red_pixels_2);   
    //get all of the red pixels in the image 
    cv::Mat all_reds;
    all_reds = red_pixels_1 | red_pixels_2;

    //according to the tutorial from the link below, we need to blur the image to reduce the noise 
    //https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html
    cv::GaussianBlur(all_reds, all_reds, cv::Size(9,9), 2, 2);

    // FIND CIRCLES IN NEW RED/ORANGE PIXEL IMAGE
    //now, let's use Hough Transforms to find any circles in the image
    cv::vector<cv::Vec3f> circles;
    double minDist = 80;
    cv::HoughCircles(all_reds, circles, CV_HOUGH_GRADIENT, 1, minDist, 100, 20, 0, 0);

    //if there are no circles found, spin robot so that it can look for the ball
    if(circles.size() == 0) {
        vel.angular.z = 0.5;
	//vel_pub.publish(vel);
    }
    else {
        //according to the docs, each circle is represented as an array in the following form:
        //circle[i] = [center_x_position, center_y_position, radius]
        //loop through each circle and find the closest one 
        cv::Point ball;
        for(int i=0; i < circles.size(); i++) {
            cv::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            //outline entire circle in green
            cv::circle(cv_ptr->image, center, radius, CV_RGB(0,255,0));
	    std::cout << radius;
            //get depth of the ball
	    int depth = cv_depth_ptr->image.at<short int>(center);
	    // DETERMINE VELOCITY OF TURTLEBOT
            //do twist first (twist left of right to keep ball in middle of frame)
            //if(ball.x < MID_X_LOW || ball.x > MID_X_HIGH) {
                //vel.angular.z = ((ball.x < MID_X_LOW) ? 0.2 : -0.2);
            }
            //now, move the robot (forward or backward) based on its distance from the ball
            //if(depth < one_meter_distance_low || depth > one_meter_distance_high) {
                //vel.linear.x = ((depth < one_meter_distance_low) ? -0.2 : 0.2);
            //}
	    //vel_pub.publish(vel);
        }
    }

    // Update GUI Window
    cv::imshow(OPENCV_WINDOW, cv_ptr->image);
    cv::waitKey(3);
    
    // Output modified video stream
    image_pub_.publish(cv_ptr->toImageMsg());
  }
};

int main(int argc, char** argv)
{
  ros::init(argc, argv, "red_ball_tracker");
  ImageConverter ic;
  ros::spin();
  return 0;
}
