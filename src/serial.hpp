#ifndef __SERIAL_HPP__
#define __SERIAL_HPP__

void serial_init(char *port_name, int baudrate, int serial_fd);
void send_pose_to_serial_1(int tracker_id, float pos_x_m, float pos_y_m, float pos_z_m,
			 float quat_x, float quat_y, float quat_z, float quat_w, float roll, float pitch, float thrust, int serial_fd);
void send_pose_to_serial_2(int tracker_id, float pos_x_m, float pos_y_m, float pos_z_m,
			 float quat_x, float quat_y, float quat_z, float quat_w, float roll, float pitch, float thrust, int serial_fd);
void send_pose_to_serial_3(int tracker_id, float pos_x_m, float pos_y_m, float pos_z_m,
			 float quat_x, float quat_y, float quat_z, float quat_w, float roll, float pitch, float thrust, int serial_fd);


#endif
