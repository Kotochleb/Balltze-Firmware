#include <Arduino.h>
#include "batt.hpp"
#include <rcl/rcl.h>
#include <rcl/error_handling.h>
#include <rclc/rclc.h>
#include <micro_ros_utilities/type_utilities.h>
#include <micro_ros_utilities/string_utilities.h>


const int batt_port[] = {BAT0_PORT, BAT1_PORT, BAT2_PORT}; 
const float batt_range[] = {MAX_VOLTAGE, MAX_VOLTAGE*2, MAX_VOLTAGE*3};

int battery_init(sensor_msgs__msg__BatteryState &batt_msg)
{
    batt_msg.header.frame_id = micro_ros_string_utilities_set(batt_msg.header.frame_id, "/batt");
    batt_msg.cell_voltage.capacity = CELL_NUM;
    batt_msg.cell_voltage.size = CELL_NUM;
    batt_msg.cell_voltage.data = (float*) malloc(CELL_NUM * sizeof(float));
    batt_msg.temperature = NAN;
    batt_msg.current = NAN;
    batt_msg.charge = NAN;
    batt_msg.capacity = NAN;
    batt_msg.design_capacity = NAN;
    batt_msg.percentage = NAN;
    batt_msg.cell_temperature.capacity = 0;
    batt_msg.cell_temperature.size = 0;
    batt_msg.cell_temperature.data = NULL;
    return 0;
}

int battery_read(sensor_msgs__msg__BatteryState &batt_msg)
{
    for(int i=0; i<CELL_NUM; i++)
    {
        int x = analogRead(batt_port[i]);
        batt_msg.cell_voltage.data[i] = (float) x*batt_range[i]/((1<<BAT_RES)-1);
    }
    batt_msg.voltage = batt_msg.cell_voltage.data[CELL_NUM-1];
    return 0;
}