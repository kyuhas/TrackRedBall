#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <math.h>
#include <geometry_msgs/Point.h>

static const std::string OPENCV_WINDOW = "Image window";

class ImageConverter
{
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  
  
public:
  ImageConverter()
    : it_(nh_)
  {
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/usb_cam/image_raw", 10, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 10);

    //publisher of ball centers
    ros::Publisher ball_center_pub = it_.advertize<geometry_msgs::Point>("BallCenter", 10);

    cv::namedWindow(OPENCV_WINDOW);
  }

  ~ImageConverter()
  {
    cv::destroyWindow(OPENCV_WINDOW);
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg)
  {
    cv_bridge::CvImagePtr cv_ptr;
    cv::Mat hsv_image;
    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      // Convert priginal image to HSV
      cv::cvtColor(cv_ptr->image, hsv_image, COLOR_BGR2HSV);
    }
    catch (cv_bridge::Exception& e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    //use thresholding to keep only red pixels
    //to find thresholded color ranges, I used this website: http://www.tydac.ch/color/
    //red seems to be the only color that wraps around, so we need two Mats to represent it
    cv::Mat red_pixels_1;
    cv::Mat red_pixels_2;
    cv::inRange(hsv_image, cv::Scalar(11, 200, 125), cv::Scalar(17, 255, 255), red_pixels);
    cv::inRange(hsv_image, cv::Scalar(170, 200, 125), cv::Scalar(180, 255, 255), red_pixels);   
    //get all of the red pixels in the image 
    cv::Mat all_reds;
    all_reds = red_pixels_1 | red_pixels_2;

    //according to the tutorial from the link below, we need to blur the image to reduce the noise 
    //https://docs.opencv.org/2.4/doc/tutorials/imgproc/imgtrans/hough_circle/hough_circle.html
    cv::GaussianBlur(all_reds, all_reds, Size(9,9), 0, 0);

    //now, let's use Hough Transforms to find any circles in the image
    cv::vector<Vec3f> circles;
    //TODO: will likely need to tweak this value. 
    //TODO: there are also 4 other params that may need to be set
    //see http://opencvexamples.blogspot.com/2013/10/hough-circle-detection.html for other params 
    double minDist = 50;
    cv::HoughCircles(all_reds, circles, CV_HOUGH_GRADIENT, 1, minDist);


    //if there are no circles found, spin robot until one is found
    if(circles.size() == 0) {
        geometry_msgs::Point empty;
        empty.x = NULL;
        empty.y = NULL;
        empty.z = NULL;
        ball_center_pub.publish(empty);
    }
    else {
        //according to the docs, each circle is represented as an array in the following form:
        //circle[i] = [center_x_position, center_y_position, radius]
        //loop through each circle and find the closest one -- stay 1 meter away from this one
        int min_depth = NULL;
        geometry_msgs:: ball = NULL;
        for(size_t i=0; i < circles.size(); i++) {
            geometry_msgs::Point center(cvRound(circles[i][0]), cvRound(circles[i][1]));
            int radius = cvRound(circles[i][2]);
            // outline center of circle in green
            cv::circle(cv_ptr->image, center, 10, CV_RGB(0,255,0));
            // outline entire circle in red
            cv::circle(cv_ptr->image, center, radius, CV_RGB(255,0,0));
            int depth = cv_ptr->image.at<short int>(center);
            if(min_depth == NULL | (min_depth != NULL && depth <= min_depth)) {
                min_depth = depth;
                ball = center;
            }
        }
        //publish ball position
        ball_center_pub.publish(ball);
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