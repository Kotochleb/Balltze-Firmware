#include <Arduino.h>
#include "imu.hpp"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>

#include "hardware/spi.h"
#include <Wire.h>

Adafruit_LSM9DS1 lsm = Adafruit_LSM9DS1(1);

void euler_to_quat(float x, float y, float z, double *q) {
  float c1 = cos((y*PI/180.0)/2.0);
  float c2 = cos((z*PI/180.0)/2.0);
  float c3 = cos((x*PI/180.0)/2.0);

  float s1 = sin((y*PI/180.0)/2.0);
  float s2 = sin((z*PI/180.0)/2.0);
  float s3 = sin((x*PI/180.0)/2.0);

  q[0] = c1 * c2 * c3 - s1 * s2 * s3;
  q[1] = s1 * s2 * c3 + c1 * c2 * s3;
  q[2] = s1 * c2 * c3 + c1 * s2 * s3;
  q[3] = c1 * s2 * c3 - s1 * c2 * s3;
}

int imu_init(sensor_msgs__msg__Imu &imu_msg)
{
  lsm.begin();

    // 1.) Set the accelerometer range
  lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_2G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_4G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_8G);
  //lsm.setupAccel(lsm.LSM9DS1_ACCELRANGE_16G);
  
    // 2.) Set the magnetometer sensitivity
  lsm.setupMag(lsm.LSM9DS1_MAGGAIN_4GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_8GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_12GAUSS);
  //lsm.setupMag(lsm.LSM9DS1_MAGGAIN_16GAUSS);

    // 3.) Setup the gyroscope
  lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_245DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_500DPS);
  //lsm.setupGyro(lsm.LSM9DS1_GYROSCALE_2000DPS);

  imu_msg.header.frame_id = micro_ros_string_utilities_set(imu_msg.header.frame_id, "/imu");
    
  return 0;
}

int imu_read(sensor_msgs__msg__Imu &imu_msg)
{
  lsm.read();
  sensors_event_t a, m, g, temp;
  lsm.getEvent(&a, &m, &g, &temp);

  double q[4];
  imu_msg.angular_velocity.x = g.gyro.z;
  imu_msg.angular_velocity.y = g.gyro.y;
  imu_msg.angular_velocity.z = g.gyro.z;
  imu_msg.linear_acceleration.x = a.acceleration.x;
  imu_msg.linear_acceleration.y = a.acceleration.y;
  imu_msg.linear_acceleration.z = a.acceleration.z;

  euler_to_quat(a.orientation.roll, a.orientation.pitch, a.orientation.heading, q);

  imu_msg.orientation.w = q[0];
  imu_msg.orientation.x = q[1];
  imu_msg.orientation.y = q[2];
  imu_msg.orientation.z = q[3];

  return 0;
}

