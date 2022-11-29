#include "ros/ros.h"
#include "std_msgs/String.h"
#include "mav_msgs/Distribution.h"
#include <sstream>
#include "auto_flight/command.h"
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Vector3.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
#include <tf2/transform_datatypes.h>
#include <tf/transform_datatypes.h>
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Core>
#include <math.h>
#include <cmath>
#include <iostream>

// Xbee Output: uav_command : roll, pitch, yaw
auto_flight::command uav1_command ;
auto_flight::command uav2_command;
auto_flight::command uav3_command;
geometry_msgs::PoseStamped uav_pose;

// From controller: raw_command
geometry_msgs::Vector3 uav1_command_raw;
geometry_msgs::Vector3 uav2_command_raw;
geometry_msgs::Vector3 uav3_command_raw;

// uav_orientation
struct uav_R{
  double roll;
  double pitch;
  double yaw;
};

uav_R uav1_R;
uav_R uav2_R;
uav_R uav3_R;

// payload control output
mav_msgs::Distribution uav_pub_output;


void command_maker(geometry_msgs::Vector3& uav_command_raw, auto_flight::command& uav_command,
 uav_R uav_orientation){
  float uav_mass = 0.7;
  Eigen::Vector3d g;
  g << 0, 0, 9.81;
  Eigen::Vector3d self_weight;
  self_weight = g*uav_mass;

  // get f scalar
  Eigen::Vector3d temp;
  temp << uav_command_raw.x , uav_command_raw.y, uav_command_raw.z;
  temp = temp + self_weight;
  double f_norm = sqrt(temp(0)*temp(0) + temp(1)*temp(1) + temp(2)*temp(2));
  // get yaw angle rotation matrix
  Eigen::AngleAxisd init_rotation_z((float)uav_orientation.yaw, Eigen::Vector3d::UnitZ());
  Eigen::Matrix3d z_rot = init_rotation_z.matrix();
  // get U
  Eigen::Vector3d U = 1/f_norm*z_rot.transpose()*temp;
  // Get command
  uav_command.roll = -asin(U(1));
  uav_command.pitch = atan(U(0)/U(2));
  uav_command.thrust = f_norm;
}

void uav1_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  uav_pose = *msg;
  // orientation
  tf::Quaternion Q(uav_pose.pose.orientation.x,uav_pose.pose.orientation.y,  uav_pose.pose.orientation.z, uav_pose.pose.orientation.w);
  tf::Matrix3x3(Q).getRPY(uav1_R.roll, uav1_R.pitch, uav1_R.yaw);
  Eigen::Quaterniond eigen_quat;
  tf::quaternionTFToEigen(Q, eigen_quat);
  eigen_quat.toRotationMatrix();
  
  command_maker(uav1_command_raw, uav1_command , uav1_R);
  std::cout << "UAV1 RPY angle (rad) : " << uav1_R.roll << " "<< uav1_R.pitch << " "<< uav1_R.yaw << std::endl;
  ROS_INFO( "UAV1_command=(R:%.2lf(rad), P:%.2lf(rad), T:%.2lf(N))"
                 ,uav1_command.roll, uav1_command.pitch, uav1_command.thrust);

}

void uav2_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  uav_pose = *msg;
  // orientation
  tf::Quaternion Q(uav_pose.pose.orientation.x,uav_pose.pose.orientation.y,  uav_pose.pose.orientation.z, uav_pose.pose.orientation.w);
  tf::Matrix3x3(Q).getRPY(uav2_R.roll, uav2_R.pitch, uav2_R.yaw);
  Eigen::Quaterniond eigen_quat;
  tf::quaternionTFToEigen(Q, eigen_quat);
  eigen_quat.toRotationMatrix();
  std::cout << "UAV2 RPY angle (rad) : " << uav2_R.roll << " "<< uav2_R.pitch << " "<< uav2_R.yaw << std::endl;
  command_maker(uav2_command_raw, uav2_command , uav2_R);
  
  ROS_INFO( "UAV2_command=(R:%.2lf(rad), P:%.2lf(rad), T:%.2lf(N))"
                 ,uav2_command.roll, uav2_command.pitch, uav2_command.thrust);

}


void uav3_cb(const geometry_msgs::PoseStamped::ConstPtr& msg){
  uav_pose = *msg;
  // orientation
  tf::Quaternion Q(uav_pose.pose.orientation.x,uav_pose.pose.orientation.y,  uav_pose.pose.orientation.z, uav_pose.pose.orientation.w);
  tf::Matrix3x3(Q).getRPY(uav3_R.roll, uav3_R.pitch, uav3_R.yaw);
  Eigen::Quaterniond eigen_quat;
  tf::quaternionTFToEigen(Q, eigen_quat);
  eigen_quat.toRotationMatrix();
  std::cout << "UAV3 RPY angle (rad) : " << uav3_R.roll << " "<< uav3_R.pitch << " "<< uav3_R.yaw << std::endl;
  command_maker(uav3_command_raw, uav3_command , uav3_R);
  ROS_INFO( "UAV3_command=(R:%.2lf(rad), P:%.2lf(rad), T:%.2lf(N))"
                 ,uav3_command.roll, uav3_command.pitch, uav3_command.thrust);

}

void control_cb(const mav_msgs::Distribution::ConstPtr& msg){
  uav1_command_raw = msg->uav1_force;
  uav2_command_raw = msg->uav2_force;
  uav3_command_raw = msg->uav3_force;
}


int main(int argc, char **argv)
{

  ros::init(argc, argv, "uav_command_distributor");

  ros::NodeHandle n;
  // subscribe uav orientation
  ros::Subscriber uav1_R_sub = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV1/pose",3,uav1_cb);
  ros::Subscriber uav2_R_sub = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV2/pose",3,uav2_cb);
  ros::Subscriber uav3_R_sub = n.subscribe<geometry_msgs::PoseStamped>("/vrpn_client_node/MAV3/pose",3,uav3_cb);
  
  // subscribe controller output
  ros::Subscriber controller_sub = n.subscribe<mav_msgs::Distribution>("/center/uav_distribution",3,control_cb);
  
  // publish uav command for xbee
  ros::Publisher uav1_command_pub = n.advertise<auto_flight::command>("transport_command_1", 1);
  ros::Publisher uav2_command_pub = n.advertise<auto_flight::command>("transport_command_2", 1);
  ros::Publisher uav3_command_pub = n.advertise<auto_flight::command>("transport_command_3", 1);

  ros::Rate loop_rate(120);

  int count = 0;
  while (ros::ok())
  {

    uav1_command_pub.publish(uav1_command);
    uav2_command_pub.publish(uav2_command);
    uav3_command_pub.publish(uav3_command);
    
    ros::spinOnce();
    loop_rate.sleep();
    ++count;
  }


  return 0;
}
