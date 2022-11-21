#include<iostream>
#include<thread>
#include"ros/ros.h"
#include<serial.hpp>

#include"thread/main_thread.h"

using namespace std;

main(int argc ,char **argv){

	ros::init(argc,argv,"ncrl_auto_flght");
	ros::Time::init();
	
	int serial_id_1 = 1;
	int serial_id_2 = 2;
	int serial_id_3 = 3;

	cout << "main start" << endl;      

	cout << "port1 init" << endl;        
	serial_init((char *)"/dev/ttyUSB0", 115200, serial_id_1);
	cout << "port2 init" << endl;        
	serial_init((char *)"/dev/ttyUSB1", 115200, serial_id_2);
	cout << "port3 init" << endl;        
	serial_init((char *)"/dev/ttyUSB2", 115200, serial_id_3);

	std::thread send_thread_1(send_thread_entry_1, serial_id_1);
	std::thread send_thread_2(send_thread_entry_2, serial_id_2);
	std::thread send_thread_3(send_thread_entry_3, serial_id_3);
	
	send_thread_1.join();
	send_thread_2.join();
	send_thread_3.join();

	return 0;
}
