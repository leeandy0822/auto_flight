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

imu_t imu;
uint8_t rc_ch7;

using std;

uint8_t generate_imu_checksum_byte(uint8_t *payload, int payload_count){
	uint8_t result = IMU_CHECKSUM_INIT_VAL;

	int i;
	for(i = 0; i < payload_count; i++)
		result ^= payload[i];

	return result;
}

int imu_decode(uint8_t *buf){
	static float x_array_uart[100];
	uint8_t recv_checksum = buf[1];
	uint8_t checksum = generate_imu_checksum_byte(&buf[3], IMU_SERIAL_MSG_SIZE - 3);
	if(checksum != recv_checksum) {
		return 1; //error detected
	}
	float roll, pitch, yaw, throttle;
    
	memcpy(&roll, &buf[2], sizeof(float)); //in ned coordinate system
	memcpy(&pitch, &buf[6], sizeof(float));
	memcpy(&yaw, &buf[10], sizeof(float));

	/* swap the order of quaternion to make the frame consistent with ahrs' rotation order */
	memcpy(&throttle, &buf[14], sizeof(float));
	memcpy(&rc_ch7, &buf[18], sizeof(int));
	//memcpy(&imu.gyrop[0], &buf[22], sizeof(float));
	
	cout << roll <<endl;



	return 0;
}

void imu_buf_push(uint8_t c){
	if(imu.buf_pos >= IMU_SERIAL_MSG_SIZE) {
		/* drop the oldest data and shift the rest to left */
		int i;
		for(i = 1; i < IMU_SERIAL_MSG_SIZE; i++) {
			imu.buf[i - 1] = imu.buf[i];
		}

		/* save new byte to the last array element */
		imu.buf[IMU_SERIAL_MSG_SIZE - 1] = c;
		imu.buf_pos = IMU_SERIAL_MSG_SIZE;
	} else {
		/* append new byte if the array boundary is not yet reached */
		imu.buf[imu.buf_pos] = c;
		imu.buf_pos++;
	}
}

int uart_thread_entry(){
	ros::NodeHandle k;
    ros::Rate loop_rate_(800);
	char c;
	imu.buf_pos = 0;

	uint8_t last_ecbf_mode = 0;
	ros::Time now_uart_time = ros::Time::now();
	ros::Time last_uart_time = ros::Time::now();


	while(ros::ok()){
		
		if(serial_getc(&c) != -1) {
			imu_buf_push(c); 
			if(imu.buf[0]=='@' && imu.buf[IMU_SERIAL_MSG_SIZE-1] == '+'){
				if(imu_decode(imu.buf)==0){

					loop_rate_.sleep();

					send_pose_to_serial(0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0);
						
					
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

