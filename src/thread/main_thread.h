#ifndef __MAIN_THREAD__
#define __MAIN_THREAD__

#include "auto_flight/ncrl_link.h"
#include "auto_flight/transportation.h"
// pc -> px4
#define NCRL_LINK_SERIAL_MSG_SIZE 44
// for check
#define NCRL_LINK_CHECKSUM_INIT_VAL 19
// px4->pc
#define NCRL_LINK_MSG_SIZE 21

#define MASS 1.39f
#define GRAVITY 9.8f


typedef struct {
	
	volatile int buf_pos;
	double deviation_acc;	
	int tracker_id;
	float px;
	float py;
	float pz;
	float qx;	
	float qy;
	float qz;
	float qw;
	float roll;
	float pitch;
	float thrust;
	uint8_t buf[44];

} transportation_t;


int send_thread_entry_1(int serial_fd);
int send_thread_entry_2(int serial_fd);
int send_thread_entry_3(int serial_fd);
void callback1(const auto_flight::transportation::ConstPtr& msg);
void callback2(const auto_flight::transportation::ConstPtr& msg);
void callback3(const auto_flight::transportation::ConstPtr& msg);

void callback(const auto_flight::transportation::ConstPtr& msg);
#endif
