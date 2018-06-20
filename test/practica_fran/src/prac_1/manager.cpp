#include "ros/ros.h"
#include "std_msgs/String.h"
#include "geometry_msgs/Twist.h"

void cmd_dirCallback(const std_msgs::String::ConstPtr& msg);
ros::Publisher pub;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "listener");
  ros::NodeHandle n;

  /**
   * The subscribe() call is how you tell ROS that you want to receive messages
   * on a given topic.  Messages are passed to a callback function, here
   * called cmd_dirCallback.  subscribe() returns a Subscriber object that you
   * must hold on to until you want to unsubscribe.  When all copies of the Subscriber
   * object go out of scope, this callback will automatically be unsubscribed from
   * this topic.
  **/
  ros::Subscriber sub = n.subscribe("cmd_dir", 1000, cmd_dirCallback);

  pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

  /**
   * ros::spin() will enter a loop, pumping callbacks.  With this version, all
   * callbacks will be called from within this thread (the main one).  ros::spin()
   * will exit when Ctrl-C is pressed, or the node is shutdown by the master.
   */
  ros::spin();
  return 0;
}

void cmd_dirCallback(const std_msgs::String::ConstPtr& msg)
{
  ROS_INFO("Received the direction command [%s]", msg->data.c_str());
  geometry_msgs::Twist sent_message;
  double linear=0, angular=0;

  if (!strcmp(msg->data.c_str(),"stop"))
  {linear=0;angular=0;}
  else if (!strcmp(msg->data.c_str(),"forward"))
  {linear=1;angular=0;}
  else if (!strcmp(msg->data.c_str(),"backward"))
  {linear=-1;angular=0;}
  else if (!strcmp(msg->data.c_str(),"turn_left"))
  {linear=0;angular=1;}
  else if (!strcmp(msg->data.c_str(),"turn_right"))
  {linear=0;angular=-1;}

  sent_message.linear.x=linear;
  sent_message.angular.z=angular;

  ROS_INFO_STREAM("Value " << sent_message.linear.x << " " << sent_message.angular.z);
  pub.publish(sent_message);

}

