#include "mbot_linux_serial.h"
#include <boost/asio.hpp>
#include <inttypes.h>
using namespace std;
using namespace boost::asio;
boost::asio::io_service iosev;
boost::asio::serial_port sp(iosev, "/dev/ttyUSB0");
boost::system::error_code err;

// union intdata
// {
//     int d;
//     unsigned char data[2];
// }set_left_rpm,set_right_rpm;

union float_data
{
    float val;
     unsigned char data[4];
}longitude,latitude,acc_x,acc_y,acc_z,gyro_x,gyro_y,gyro_z,Lg,Lt;

bool imu_Init(float Longitude,float Latitude)
{
    unsigned char imu_init_buf[16];
    sp.set_option(serial_port::baud_rate(115200));
    sp.set_option(serial_port::flow_control(serial_port::flow_control::none));
    sp.set_option(serial_port::parity(serial_port::parity::none));
    sp.set_option(serial_port::stop_bits(serial_port::stop_bits::one));
    sp.set_option(serial_port::character_size(8));
    longitude.val =Longitude;
    latitude.val = Latitude;
    imu_init_buf[0]=0xaa;
    imu_init_buf[1] = longitude.data[3];
    imu_init_buf[2] = longitude.data[2];
    imu_init_buf[3] = longitude.data[1];
    imu_init_buf[4] = longitude.data[0];

    imu_init_buf[5] = latitude.data[3];
    imu_init_buf[6] = latitude.data[2];
    imu_init_buf[7] = latitude.data[1];
    imu_init_buf[8] = latitude.data[0];

    imu_init_buf[9] = 0x01;
    for(int i=10;i<=14;i++)
    {
        imu_init_buf[i] = 0x00;
    }
    imu_init_buf[15] = imu_check(imu_init_buf);    
    // for(int i =0;i<sizeof(imu_init_buf);i++)
    // {
    //     printf("imu_init_buf[%d] = 0x%"PRIx64"\n",i, imu_init_buf[i]);
    // }
    boost::asio::write(sp, boost::asio::buffer(imu_init_buf));
    return true;
}

bool imu_receive(struct Imu_Data* imu_data)
{
    unsigned char imu_receive_data[48];
    boost::asio::read(sp,buffer(imu_receive_data));
    for (int i = 0; i < 4; i++)
    {
        gyro_x.data[i] = imu_receive_data[i+11];
        gyro_y.data[i] = imu_receive_data[i+15];
        gyro_z.data[i] = imu_receive_data[i+19];
        acc_x.data[i]   = imu_receive_data[i+23];
        acc_y.data[i]   = imu_receive_data[i+27];
        acc_z.data[i]   = imu_receive_data[i+31];
        // Lg.data[i]     = imu_receive_data[i+35];
        // Lt.data[i]      = imu_receive_data[i+39];
    }
    imu_data->acc_x = acc_x.val;
    imu_data->acc_y  = acc_y.val;
    imu_data->acc_z  = acc_z.val;
    imu_data->gyro_x = gyro_x.val;
    imu_data->gyro_y = gyro_y.val;
    imu_data->gyro_z = gyro_z.val;
    imu_data->lg = Lg.val;
    imu_data->lt = Lt.val;
    return true;
}

unsigned char imu_check(unsigned char* buf)
{
    unsigned char result=0;
    for (size_t i = 0; i < 15; i++)
    {
        result+= buf[i];
        // printf("buf[%d] = 0x%"PRIx64"\n", i,buf[i]);
        // printf("result = 0x%"PRIx64"\n", result);
    }
    return result;
}