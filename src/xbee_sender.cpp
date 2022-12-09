#include "ros/ros.h"
#include "std_msgs/String.h"
#include <geometry_msgs/PoseStamped.h>
#include "auto_flight/transportation.h"
#include "auto_flight/command.h"


auto_flight::transportation transport_msg_1;
auto_flight::command command_msg_1;
geometry_msgs::PoseStamped optitrack_data_1;

auto_flight::transportation transport_msg_2;
auto_flight::command command_msg_2;
geometry_msgs::PoseStamped optitrack_data_2;

auto_flight::transportation transport_msg_3;
auto_flight::command command_msg_3;
geometry_msgs::PoseStamped optitrack_data_3;

void optitrack_cb_1(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  optitrack_data_1 = *msg;
  transport_msg_1.px = optitrack_data_1.pose.position.x;
  transport_msg_1.py = optitrack_data_1.pose.position.y;
  transport_msg_1.pz = optitrack_data_1.pose.position.z;
  transport_msg_1.qx = optitrack_data_1.pose.orientation.x;
  transport_msg_1.qy = optitrack_data_1.pose.orientation.y;
  transport_msg_1.qz = optitrack_data_1.pose.orientation.z;
  transport_msg_1.qw = optitrack_data_1.pose.orientation.w;
}

void optitrack_cb_2(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  optitrack_data_2 = *msg;
  transport_msg_2.px = optitrack_data_2.pose.position.x;
  transport_msg_2.py = optitrack_data_2.pose.position.y;
  transport_msg_2.pz = optitrack_data_2.pose.position.z;
  transport_msg_2.qx = optitrack_data_2.pose.orientation.x;
  transport_msg_2.qy = optitrack_data_2.pose.orientation.y;
  transport_msg_2.qz = optitrack_data_2.pose.orientation.z;
  transport_msg_2.qw = optitrack_data_2.pose.orientation.w;
}
void optitrack_cb_3(const geometry_msgs::PoseStamped::ConstPtr& msg)
{
  optitrack_data_3 = *msg;
  transport_msg_3.px = optitrack_data_3.pose.position.x;
  transport_msg_3.py = optitrack_data_3.pose.position.y;
  transport_msg_3.pz = optitrack_data_3.pose.position.z;
  transport_msg_3.qx = optitrack_data_3.pose.orientation.x;
  transport_msg_3.qy = optitrack_data_3.pose.orientation.y;
  transport_msg_3.qz = optitrack_data_3.pose.orientation.z;
  transport_msg_3.qw = optitrack_data_3.pose.orientation.w;
}

void command_cb_1(const auto_flight::command::ConstPtr& msg)
{
  command_msg_1 = *msg;
  // degree
  transport_msg_1.roll = command_msg_1.roll;
  // degree
  transport_msg_1.pitch = command_msg_1.pitch;
  // newton
  transport_msg_1.thrust = command_msg_1.thrust;
}

void command_cb_2(const auto_flight::command::ConstPtr& msg)
{
  command_msg_2 = *msg;
  // degree
  transport_msg_2.roll = command_msg_2.roll;
  // degree
  transport_msg_2.pitch = command_msg_2.pitch;
  // newton
  transport_msg_2.thrust = command_msg_2.thrust;
}
void command_cb_3(const auto_flight::command::ConstPtr& msg)
{
  command_msg_3 = *msg;
  // degree
  transport_msg_3.roll = command_msg_3.roll;
  // degree
  transport_msg_3.pitch = command_msg_3.pitch;
  // newton
  transport_msg_3.thrust = command_msg_3.thrust;
}
int main(int argc, char **argv)
{

  ros::init(argc, argv, "xbee_sender");
  ros::NodeHandle n;

  ros::Publisher transport_pub_1 = n.advertise<auto_flight::transportation>("pc_to_pixhawk_1",1);
  ros::Subscriber optitrack_sub_1 = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV1/pose",3,optitrack_cb_1);
  ros::Subscriber command_sub_1 = n.subscribe<auto_flight::command>("transport_command_1",1, command_cb_1);
  
  ros::Publisher transport_pub_2 = n.advertise<auto_flight::transportation>("pc_to_pixhawk_2",1);
  ros::Subscriber optitrack_sub_2 = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV2/pose",3,optitrack_cb_2);
  ros::Subscriber command_sub_2 = n.subscribe<auto_flight::command>("transport_command_2",1, command_cb_2);
  
  ros::Publisher transport_pub_3 = n.advertise<auto_flight::transportation>("pc_to_pixhawk_3",1);
  ros::Subscriber optitrack_sub_3 = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV3/pose",3,optitrack_cb_3);
  ros::Subscriber command_sub_3 = n.subscribe<auto_flight::command>("transport_command_3",1, command_cb_3);

  ros::Rate loop_rate(100);
  transport_msg_1.tracker_id = 1;
  transport_msg_2.tracker_id = 2;
  transport_msg_3.tracker_id = 3;


  while (ros::ok())
  {

    transport_pub_1.publish(transport_msg_1);
    transport_pub_2.publish(transport_msg_2);
    transport_pub_3.publish(transport_msg_3);

    ros::spinOnce();

    loop_rate.sleep();
  }


  return 0;
}