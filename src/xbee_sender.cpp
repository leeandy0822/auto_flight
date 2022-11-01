#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include "auto_flight/transportation.h"
#include "auto_flight/command.h"


auto_flight::transportation transport_msg;
auto_flight::command command_msg;
geometry_msgs::PoseStamped optitrack_data;

void optitrack_cb(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  optitrack_data = *msg;
  transport_msg.px = optitrack_data.pose.position.x;
  transport_msg.py = optitrack_data.pose.position.y;
  transport_msg.pz = optitrack_data.pose.position.z;
  transport_msg.qx = optitrack_data.pose.orientation.x;
  transport_msg.qy = optitrack_data.pose.orientation.y;
  transport_msg.qz = optitrack_data.pose.orientation.z;
  transport_msg.qw = optitrack_data.pose.orientation.w;
}

void command_cb(const auto_flight::command::ConstPtr& msg)
{
  command_msg = *msg;
  // degree
  transport_msg.roll = command_msg.roll;
  // degree
  transport_msg.pitch = command_msg.pitch;
  // newton
  transport_msg.thrust = command_msg.thrust;
}

int main(int argc, char **argv)
{

  ros::init(argc, argv, "xbee_sender");
  ros::NodeHandle n;

  ros::Publisher transport_pub = n.advertise<auto_flight::transportation>("pc_to_pixhawk",1);
  ros::Subscriber optitrack_sub = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV1/pose",3,optitrack_cb);
  ros::Subscriber command_sub = n.subscribe<auto_flight::command>("transport_command",1, command_cb);

  ros::Rate loop_rate(400);
  // default = 1
  transport_msg.tracker_id = 1;
  // degree
  transport_msg.roll = 0;
  // degree
  transport_msg.pitch = 0;
  // newton
  transport_msg.thrust = 5;
  while (ros::ok())
  {

    transport_pub.publish(transport_msg);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}
