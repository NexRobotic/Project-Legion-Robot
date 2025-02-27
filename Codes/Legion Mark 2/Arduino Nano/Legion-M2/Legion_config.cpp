#include "Legion_config.h"
#include <math.h> 
#include <Arduino.h>

// Servo settings
Servo servo[4][3];
const int servo_channels[4][3] = { {0, 1, 2}, {4, 5, 6}, {8, 9, 10}, {12, 13, 14} };

// Robot size constants original
// const float length_a = 55;
// const float length_b = 77.5;
// const float length_c = 27.5;
// const float length_side = 71;
// const float z_absolute = -28;

// Robot size constants original M2
const float length_a = 55;
const float length_b = 77.5;
const float length_c = 27.5;
const float length_side = 113;
const float z_absolute = -28;
// Movement constants

float z_default = -60, z_up = -30, z_boot = z_absolute; //-60=z_default
float x_default = 70, x_offset = 0;
float y_start = 0, y_step = 40;
float y_default = 62;
float spot_turn_speed = 4;
float leg_move_speed = 8;
float body_move_speed = 3;
float stand_seat_speed = 1;
float KEEP = 255;
float pi = 3.1415926;

// Turn constants
const float temp_a = sqrt(pow(2 * x_default + length_side, 2) + pow(y_step, 2));
const float temp_b = 2 * (y_start + y_step) + length_side;
const float temp_c = sqrt(pow(2 * x_default + length_side, 2) + pow(2 * y_start + y_step + length_side, 2));
const float temp_alpha = acos((pow(temp_a, 2) + pow(temp_b, 2) - pow(temp_c, 2)) / 2 / temp_a / temp_b);
const float turn_x1 = (temp_a - length_side) / 2;
const float turn_y1 = y_start + y_step / 2;
const float turn_x0 = turn_x1 - temp_b * cos(temp_alpha);
const float turn_y0 = temp_b * sin(temp_alpha) - turn_y1 - length_side;

// Variables for movement
volatile float site_now[4][3];
volatile float site_expect[4][3];
float temp_speed[4][3];
float move_speed;
float speed_multiple = 1;
volatile int rest_counter;

const int motorPinA1 = 2;
const int motorPinA2 = 3;
const int motorPinB1 = 4;
const int motorPinB2 = 5;
const int enA = 6;
const int enB = 9;


