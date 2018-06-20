#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"
#include <stdlib.h>
#include "sensor_msgs/LaserScan.h"
#include <math.h>

void processScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
ros::Publisher pub;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "auto2");
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  Messages are passed to a callback function, here
   * called cmd_dirCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
  **/
  ros::Subscriber sub = n.subscribe("base_scan", 1000,processScanCallback);

  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  return 0;
}

void processScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{

  geometry_msgs::Twist sent_message;
  double linear=0, angular=0;
  double pi=3.141592654;
  double alpha=0, beta=-pi/2.0;
  double A=1, B=1;




    int n_ranges=msg->ranges.size();
     std::vector<float>::const_iterator min_it=
           std::min_element(msg->ranges.begin(),msg->ranges.end());
     double nearest = *min_it;
     int pos=std::distance(msg->ranges.begin(),min_it);

     ROS_INFO_STREAM("Medidas Totales: "<<n_ranges);
     ROS_INFO_STREAM("Obstaculo mas cercano en: "<< nearest << "en el vector: " << pos);




     if (nearest <= 0.5)
     {
         A = nearest/0.5;
         alpha = (pos*(270.0/1081.0)-135.0)*(pi/180.0);
         beta = pi/2.0-alpha;
         linear =1-sin(beta)/A;
         angular = -B*cos(beta);

     }
     else
     {
         linear=1;
         angular=0;
     }



  sent_message.linear.x=linear;
  sent_message.angular.z=angular;

std::cout<<"Value " << sent_message.linear.x << " " << sent_message.angular.z <<std::endl;
std::cout<<"A: " << A << " B: " << B<<std::endl;
std::cout<<"Alpha: " << alpha << " Beta: " << beta<<std::endl;
std::cout<<"sin(beta)= " << sin(beta) << "cos(beta)= " << cos(beta)<<std::endl;
  pub.publish(sent_message);

}

