#include "ros/ros.h"
#include "std_msgs/String.h"
#include "sensor_msgs/NavSatFix.h"
//added from the task
#include "nav_msgs/Odometry.h"
#include <math.h>
//lack of motivation resulted in not implementing a matrix library, e.g., eigen...
//custom message
#include "lla2enu/Dist.h"

class pub_sub
{
  private:
    ros::NodeHandle _n;
    ros::Subscriber _sub_obs, _sub_front;
    ros::Publisher _pub;
    float _x_front = NAN;
    float _y_front = NAN;
    float _z_front = NAN;
    float _x_obs   = NAN;
    float _y_obs   = NAN;
    float _z_obs   = NAN;
    lla2enu::Dist _dist;
    
  public:
    
    pub_sub()
    {
      _sub_front = _n.subscribe("/rechatter_front", 1000, &pub_sub::grabber_front, this);
      _sub_front = _n.subscribe("/rechatter_obs", 1000, &pub_sub::grabber_obs, this);
      _pub = _n.advertise<lla2enu::Dist>("/rechatter_dist", 1);
    }

    void distance_calc()
    {
      float x = _x_front - _x_obs;
      float y = _y_front - _y_obs;
      float z = _z_front - _z_obs;

      _dist.dist = sqrt(x*x+y*y+z*z);

      //Query, whether there already exists a distance to be calculated


      _pub.publish(_dist);
    }

    void grabber_front(const nav_msgs::Odometry::ConstPtr& msg)
    {
      _x_front = msg->pose.pose.position.x;
      _y_front = msg->pose.pose.position.y;
      _z_front = msg->pose.pose.position.z;

      this.distance_calc();
    }

    void grabber_obs(const nav_msgs::Odometry::ConstPtr& msg)
    {
      _x_obs = msg->pose.pose.position.x;
      _y_obs = msg->pose.pose.position.y;
      _z_obs = msg->pose.pose.position.z;
    }
};

int main(int argc, char **argv)
{
 	ros::init(argc, argv, "distance");
 	pub_sub my_pub_sub;
 	ros::spin();
 	return 0;
}
