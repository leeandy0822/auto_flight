#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <serial.hpp>
#include <mutex>
#include <cmath>
#include <vector>
#include <string>

#include "main_thread.h"


using namespace std;

NCRL_LINK::NCRL_LINK(){
	rx_data.buf_pos = 0;
	pub = n.advertise<auto_flight::ncrl_link>("pixhawk_to_pc",1);
}

// void NCRL_LINK::publisher(){

// 	auto_flight::transportation pub_data;
// 	pub_data.px = rx_data.px;
// 	pub_data.py = rx_data.py;
// 	pub_data.pz = rx_data.pz;
// 	pub_data.qx = rx_data.qx;
// 	pub_data.qy = rx_data.qy;	
// 	pub_data.qz = rx_data.qz;	
// 	pub_data.qw = rx_data.qw;	
// 	pub_data.roll = rx_data.roll;	
// 	pub_data.pitch = rx_data.pitch;	
// 	pub_data.thrust = rx_data.thrust;	

// 	pub.publish(pub_data);
// }

// uint8_t NCRL_LINK::generate_ncrl_link_checksum_byte(uint8_t *payload, int payload_count){
// 	uint8_t result = NCRL_LINK_CHECKSUM_INIT_VAL;
// 	int i;
// 	for(i = 0; i < payload_count; i++)
// 		result ^= payload[i];

// 	return result;
// }

// int NCRL_LINK::ncrl_link_decode(uint8_t *buf){
// 	uint8_t recv_checksum = buf[1];
// 	uint8_t checksum = generate_ncrl_link_checksum_byte(&buf[3], NCRL_LINK_MSG_SIZE - 3);
// 	if(checksum != recv_checksum) {
// 		return 1; //error detected
// 	}
//     memcpy(&rx_data.mode, &buf[2], sizeof(char));
// 	memcpy(&rx_data.aux_info, &buf[3], sizeof(char)); //in ned coordinate system
// 	memcpy(&rx_data.data1, &buf[4], sizeof(float)); //in ned coordinate system
// 	memcpy(&rx_data.data2, &buf[8], sizeof(float));
// 	memcpy(&rx_data.data3, &buf[12], sizeof(float));
// 	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
// 	memcpy(&rx_data.data4, &buf[16], sizeof(float));

// 	cout << "recieve data" << endl;
// 	cout << rx_data.data1 << endl;
// 	cout << rx_data.data2 << endl;
// 	cout << rx_data.data3 << endl;

// 	return 0;
// }

// void NCRL_LINK::ncrl_link_buf_push(uint8_t c){
// 	if(rx_data.buf_pos >= NCRL_LINK_MSG_SIZE) {
// 		/* drop the oldest data and shift the rest to left */
// 		int i;
// 		for(i = 1; i < NCRL_LINK_MSG_SIZE; i++) {
// 			rx_data.buf[i - 1] = rx_data.buf[i];
// 		}

// 		/* save new byte to the last array element */
// 		rx_data.buf[NCRL_LINK_MSG_SIZE - 1] = c;
// 		rx_data.buf_pos = NCRL_LINK_MSG_SIZE;
// 	} else {
// 		/* append new byte if the array boundary is not yet reached */
// 		rx_data.buf[rx_data.buf_pos] = c;
// 		rx_data.buf_pos++;
// 	}
// }


// int receive_thread_entry(){
//     ros::Rate loop_rate_(800);
// 	char c;
// 	NCRL_LINK ncrl_link;

// 	while(ros::ok()){
// 		if(serial_getc(&c) != -1) {
// 			ncrl_link.ncrl_link_buf_push(c); 
// 			if(ncrl_link.rx_data.buf[0]=='@' && ncrl_link.rx_data.buf[NCRL_LINK_MSG_SIZE-1] == '+'){
// 				if(ncrl_link.ncrl_link_decode(ncrl_link.rx_data.buf)==0){
// 					ncrl_link.publisher();
// 					loop_rate_.sleep();
// 				}
// 			}
// 		}
// 	}
// 	return 0;

// }

transportation_t tx_data;

void callback(const auto_flight::transportation::ConstPtr& msg){
	tx_data.tracker_id = msg->tracker_id;
	tx_data.px = msg->px;
	tx_data.py = msg->py;
	tx_data.pz = msg->pz;
	tx_data.qx = msg->qx;
	tx_data.qy = msg->qy;
	tx_data.qz = msg->qz;
	tx_data.qw = msg->qw;
	tx_data.roll = msg->roll;
	tx_data.pitch = msg->pitch;	
	tx_data.thrust = msg->thrust;	

}

int send_thread_entry(){
   	ros::Rate loop_rate(100);
	ros::NodeHandle nh;
	ros::Subscriber sub;

	sub = nh.subscribe<auto_flight::transportation>("pc_to_pixhawk",1,callback);
	
	void callback(const auto_flight::transportation::ConstPtr& msg);
	
	/* init tx_data */
	tx_data.tracker_id = 1;
	tx_data.px = 1;
	tx_data.py = 2;
	tx_data.pz = 3;
	tx_data.qx = 0;
	tx_data.qy = 0;
	tx_data.qz = 0;
	tx_data.qw = -1;
	tx_data.roll = 0.1;
	tx_data.pitch = 0.01;	
	tx_data.thrust = 0.3;	

	while(ros::ok()){
		
		send_pose_to_serial(tx_data.tracker_id, tx_data.px , tx_data.py, tx_data.pz,
			 tx_data.qx, tx_data.qy, tx_data.qz, tx_data.qw, \
			 tx_data.roll, tx_data.pitch, tx_data.thrust);

		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;

}

