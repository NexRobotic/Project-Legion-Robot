#ifndef LEGION_CONFIG_H
#define LEGION_CONFIG_H

#include <Servo.h>

// Servo settings
extern Servo servo[4][3];
extern const int servo_channels[4][3];

// Robot size constants
extern const float length_a, length_b, length_c, length_side, z_absolute;

// Movement constants
extern  float z_default, z_up, z_boot, x_default, x_offset, y_start, y_step, y_default;
extern  float spot_turn_speed, leg_move_speed, body_move_speed, stand_seat_speed;
extern  float KEEP;
extern  float pi;

// Turn constants
extern const float temp_a, temp_b, temp_c, temp_alpha;
extern const float turn_x1, turn_y1, turn_x0, turn_y0;

// Variables for movement
extern volatile float site_now[4][3];
extern volatile float site_expect[4][3];
extern float temp_speed[4][3];
extern float move_speed;
extern float speed_multiple;
extern volatile int rest_counter;

extern const int motorPinA1 ;
extern const int motorPinA2 ;
extern const int motorPinB1 ;
extern const int motorPinB2 ;
extern const int enA ;
extern const int enB ;

#endif
