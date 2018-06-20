#include "ros/ros.h"
#include "std_msgs/String.h"
#include <sstream>
#include <stdlib.h>
#include "sensor_msgs/LaserScan.h"

void processScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg);
ros::Publisher pub;

int main(int argc, char **argv)
{
  /**
   * The ros::init() function needs to see argc and argv; the third argument is the name of the * node.
   * You must call ros::init() before using any other part of the ROS system.
   */
  ros::init(argc, argv, "direction_auto");

  /**
   * NodeHandle is the main access point to communications with the ROS system.
   * The first NodeHandle will initialize the node; the last NodeHandle destructed will close down the node.
   */
  ros::NodeHandle n;

  /**
   * The advertise() function is how you tell ROS that you want to publish on a given topic 
   * name. advertise() returns a Publisher object which allows you to publish messages on that  * topic through a call to publish(). Once all copies of the returned Publisher object are destroyed, the topic will be automatically unadvertised.
   *
   * The second parameter to advertise() is the size of the message queue used for publishing 
   * messages. 
   */
   ros::Subscriber sub = n.subscribe("base_scan", 1000, processScanCallback);
   pub = n.advertise<std_msgs::String>("cmd_dir", 1000);

  /**
  * A ros::Rate object to specify the frequency that you would like to loop at
  * It works with a call to Rate::sleep() at the end of the loop  
  **/
  ros::Rate loop_rate(1);

  ros::spin();

  return 0;
}


void processScanCallback(const sensor_msgs::LaserScan::ConstPtr& msg)
{
  //printf(msg->ranges[540])
  //pub.publish(msg->ranges[540]();

      std_msgs::String message;
      std::stringstream ss;

      if(msg->ranges[540]>=2)
      {
              ss<<"forward";
      }
      else  if (msg->ranges[360]>=2)
      {
          ss<<"turn_right";
      }
       else
      {
           ss<<"turn_left";
      }

      message.data= ss.str();




      int n_ranges=msg->ranges.size();
      std::vector<float>::const_iterator min_it=
              std::min_element(msg->ranges.begin(),msg->ranges.end());
      double nearest = *min_it;
      int pos=std::distance(msg->ranges.begin(),min_it);

      ROS_INFO_STREAM("Medidas Totales"<<n_ranges);
      ROS_INFO_STREAM("Obstaculo mas cercano en: "<< nearest << "en el vector");
      ROS_INFO("%s",message.data.c_str());

      pub.publish(message);

}

