#include "ros/ros.h"
#include <iostream>
#include <sstream>
#include <serial.hpp>
#include <mutex>
#include <cmath>
#include <vector>
#include <string>
#include <iterator>
#include "main_thread.h"

ncrl_link_t ncrl_link;
uint8_t rc_ch7;

using namespace std;

uint8_t generate_ncrl_link_checksum_byte(uint8_t *payload, int payload_count){
	uint8_t result = NCRL_LINK_CHECKSUM_INIT_VAL;
	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

int ncrl_link_decode(uint8_t *buf){
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_ncrl_link_checksum_byte(&buf[3], NCRL_LINK_MSG_SIZE - 3);
	if(checksum != recv_checksum) {
		return 1; //error detected
	}
    memcpy(&ncrl_link.mode, &buf[2], sizeof(char));
	memcpy(&ncrl_link.aux_info, &buf[3], sizeof(char)); //in ned coordinate system
	memcpy(&ncrl_link.data1, &buf[4], sizeof(float)); //in ned coordinate system
	memcpy(&ncrl_link.data2, &buf[8], sizeof(float));
	memcpy(&ncrl_link.data3, &buf[12], sizeof(float));
	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
	memcpy(&ncrl_link.data4, &buf[16], sizeof(float));
	cout << ncrl_link.mode << endl;
	cout << ncrl_link.aux_info << endl;
	cout << ncrl_link.data1 << endl;
	cout << ncrl_link.data4 << endl;
	return 0;
}

void ncrl_link_buf_push(uint8_t c){
	if(ncrl_link.buf_pos >= NCRL_LINK_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < NCRL_LINK_MSG_SIZE; i++) {
			ncrl_link.buf[i - 1] = ncrl_link.buf[i];
		}

		/* save new byte to the last array element */
		ncrl_link.buf[NCRL_LINK_MSG_SIZE - 1] = c;
		ncrl_link.buf_pos = NCRL_LINK_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		ncrl_link.buf[ncrl_link.buf_pos] = c;
		ncrl_link.buf_pos++;
	}
}

int uart_thread_entry(){
	ros::NodeHandle k;
    ros::Rate loop_rate_(800);
	char c;
	ncrl_link.buf_pos = 0;

	uint8_t last_ecbf_mode = 0;
	ros::Time now_uart_time = ros::Time::now();
	ros::Time last_uart_time = ros::Time::now();


	while(ros::ok()){
		
		if(serial_getc(&c) != -1) {
			ncrl_link_buf_push(c); 
			if(ncrl_link.buf[0]=='@' && ncrl_link.buf[NCRL_LINK_MSG_SIZE-1] == '+'){
				if(ncrl_link_decode(ncrl_link.buf)==0){
					loop_rate_.sleep();
					// mode, aux_info, data1, data2, data3, data4
					send_pose_to_serial('u','r',0.9,2.22,3.5,90);
				}
			}
		}
		//ros::spinOnce();
	}
	return 0;

}
int ros_thread_entry(){
   ros::Rate loop_rate(1600);

	while(ros::ok()){
		loop_rate.sleep();
		ros::spinOnce();
	}
	return 0;

}

