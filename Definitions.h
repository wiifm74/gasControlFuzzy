#ifndef DEFINITIONS_H_

#define DEFINITIONS_H_

#define ONE_WIRE_BUS 2                              // Data wire is plugged into port 2 on the Arduino
#define ONE_WIRE_PWR 3                              // Use GPIO pins for power/ground to simplify the wiring
#define ONE_WIRE_GND 4                              // Use GPIO pins for power/ground to simplify the wiring

#define PUMP_WIRE_PWR 5                             // Use GPIO pins for power/ground to simplify the wiring
#define PUMP_WIRE_GND 6                             // Use GPIO pins for power/ground to simplify the wiring

#define BUZZER_WIRE_PWR 9

#define BUZZER_FREQUENCY 750

#define BUFFER_SIZE 32
#define DISPLAY_TARGET_DECIMALS 0
#define DISPLAY_ACTUAL_DECIMALS 1

//#define OFF 0x0                                     // These #defines make it easy to set the backlight color
#define RED 0x1
#define GREEN 0x2
#define YELLOW 0x3
#define BLUE 0x4
#define VIOLET 0x5
#define TEAL 0x6
#define WHITE 0x7

#define SENSOR_PRECISION 12

#define READ_TEMP_SENSORS_EVERY 1000
#define READ_USER_INPUT_EVERY 20
#define COMPUTE_AUTO_EVERY 2000
#define WRITE_DISPLAY_EVERY 50
#define CHANGE_AUTO_EVERY 1500
#define UPDATE_PUMP_EVERY 20

#define MIN_TO_MS 60000

#endif
