#ifndef __MAIN_THREAD__
#define __MAIN_THREAD__

// pc -> px4
#define NCRL_LINK_SERIAL_MSG_SIZE 22
// for check
#define NCRL_LINK_CHECKSUM_INIT_VAL 19
// px4->pc
#define NCRL_LINK_MSG_SIZE 21

#define MASS 1.39f
#define GRAVITY 9.8f


typedef struct {
	
	float acc[3];

	float gyrop[3];

	volatile int buf_pos;

	double deviation_acc;	

	uint8_t buf[];
	char mode;
	char aux_info;
	float data1;
	float data2;
	float data3;
	float data4;
	
} ncrl_link_t ;


int uart_thread_entry();

int ros_thread_entry();

uint8_t generate_ncrl_link_checksum_byte(uint8_t *, int);

int ncrl_link_decode(uint8_t *);

void ncrl_link_buf_push(uint8_t);

#endif
