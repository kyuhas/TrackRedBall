#include <ros/ros.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Twist.h> 

const double EMPTY = -1;
const double MID_X = 320;
const double one_meter_distance = 1; //TODO: update this with correct value.

class DetermineBallVelocity
{
  ros::NodeHandle n;
  ros::Subscriber centerPoints;
  ros::Publisher vel_pub;
  
public:
  DetermineBallVelocity()
  {
    // Subscribe to /BallCenterPoints
    centerPoints = n.subscribe("/BallCenterPoints", 10, 
      &DetermineBallVelocity::determineVel, this);
    // Publish command velocities
    vel_pub = n.advertise<geometry_msgs::Twist>("/cmd_vel_mux/input/teleop", 100);
  }

  ~DetermineBallVelocity()
  {
  }

  void determineVel(geometry_msgs::Point msg)
  {
    geometry_msgs::Twist vel; 
    if(msg.x == EMPTY && msg.y == EMPTY) {
        //spin robot until it finds the ball
        vel.angular.z = 1;
    }
    else{
        //do twist first (twist left of right to keep ball in middle of frame)
        if(msg.x != MID_X) {
            vel.angular.z = ((msg.x < MID_X) ? 0.1 : -0.1);
        }
        //now, move the robot (forward or backward) based on its distance from the ball
        if(msg.z != one_meter_distance) {
            //vel.linear.x = ((msg.z < one_meter_distance) ? 0.1 : -0.1);
        }
    }
    //publish the message
    //std::cout << "Vel: " << vel.angular.z << vel.linear.x;
    vel_pub.publish(vel);
  }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "determine_velocity");
    DetermineBallVelocity dv;
    ros::spin();
    return 0;
}
