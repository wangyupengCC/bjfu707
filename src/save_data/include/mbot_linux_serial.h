#ifndef MBOT_LINUX_SERIAL_H
#define MBOT_LINUX_SERIAL_H
#define BOOST_SYSTEM_NO_DEPRECATED
#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include <stdint.h>
struct Imu_Data
{
    float acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,lg,lt;
};
bool mbot_serial_init();
void mbotSendData(std::stringbuf &buf);
void mbot_check(unsigned char* buffer, int size,unsigned char &byte);
void robot_contorl(float left_rpm,float right_rpm);

bool imu_Init(float Longitude,float Latitude);
bool imu_receive(struct Imu_Data* imu_data);
unsigned char imu_check( unsigned char* buf);
#endif
