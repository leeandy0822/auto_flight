#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include "auto_flight/transportation.h"
#include "auto_flight/command.h"


auto_flight::command command_msg;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "command_publisher");
  ros::NodeHandle n;

  ros::Publisher command_pub = n.advertise<auto_flight::command>("transport_command",1);
  ros::Rate loop_rate(120);

  // default = 1
  command_msg.roll = 0;
  command_msg.pitch = 0;
  command_msg.thrust = 5;

  while (ros::ok())
  {

    command_pub.publish(command_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
