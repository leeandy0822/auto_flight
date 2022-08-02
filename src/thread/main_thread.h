#ifndef __MAIN_THREAD__
#define __MAIN_THREAD__

#define IMU_SERIAL_MSG_SIZE 21
#define IMU_CHECKSUM_INIT_VAL 19
#define FSM_MSG_SIZE 21

#define MASS 1.39f
#define GRAVITY 9.8f


typedef struct {
	
	float acc[3];

	float gyrop[3];

	volatile int buf_pos;

	double deviation_acc;	

	uint8_t buf[];
	
} imu_t ;


int uart_thread_entry();

int ros_thread_entry();

uint8_t generate_imu_checksum_byte(uint8_t *, int);

int imu_decode(uint8_t *);

void imu_buf_push(uint8_t);

#endif
