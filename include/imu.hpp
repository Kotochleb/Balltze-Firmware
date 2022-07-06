#ifndef IMU_HPP
#define IMU_HPP

#include <sensor_msgs/msg/imu.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_LSM9DS1.h>

int imu_init(sensor_msgs__msg__Imu &imu_msg);
int imu_read(sensor_msgs__msg__Imu &imu_msg);

#endif // IMU_HPP