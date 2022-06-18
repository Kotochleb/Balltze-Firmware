#ifndef BATTERY_HPP
#define BATTERY_HPP

#include <sensor_msgs/msg/battery_state.h>
#define MAX_VOLTAGE (float)3.3
#define CELL_NUM 3
#define BAT0_PORT A0
#define BAT1_PORT A1
#define BAT2_PORT A2
#define BAT_RES 10


int battery_init(sensor_msgs__msg__BatteryState &batt_msg);
int battery_read(sensor_msgs__msg__BatteryState &batt_msg);


#endif // BATTERY_HPP