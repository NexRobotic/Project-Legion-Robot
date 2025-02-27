//Mark 2 : Wheel on legion go Round and Round


#include <Servo.h>
#include <FlexiTimer2.h>
#include "Legion_config.h"
#include "Legion.h"
#include <Adafruit_PWMServoDriver.h>

// Create a PCA9685 object
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver();

// Servo configuration
#define SERVOMIN 150  // Minimum pulse length count
#define SERVOMAX 600  // Maximum pulse length count

//DIctionary
#define FORWARD 'f'
#define BACKWARD 'b'
#define LEFT 'l'
#define RIGHT 'r'
#define STOP's'
#define SIT 'i'

#define HEIGHTUP 'u'
#define HEIGHTDOWN'v'

#define MOVEFORWARD 'w'
#define MOVEBACKWARD 'x'
#define MOVELEFT 'a'
#define MOVERIGHT 'd'


#define HANDWAVE 'e'
#define HANDSHAKE 'g'

 #define DRIVEMODE 'h'
// #define  'j'
// #define  'k'
// #define  'm'
// #define  'n'
// #define  'o'
// #define  'p'
// #define  'q'
// #define  't'
// #define  'Y'
// #define  'z'

//DIctionary


void RC_Control();

const int channelPins[] = { A1, A2, A3, A4, A5, A6 };
const int numChannels = 6;

unsigned long pulseWidths[numChannels];
int channelValues[numChannels];


// Threshold values for mizn and max pulse widths
const int minValue = 1000;
const int maxValue = 1800;
unsigned long TIME_INTERVAL = 35000;  // Timeout in microseconds (adjust based on your RC receiver)

Legion legion;

bool state = true;
char cmd = STOP;
bool check_service = false;

/* Servos --------------------------------------------------------------------*/
//define 12 servos for 4 legs

void setup() {
  Serial.begin(115200);
  Serial.println("Robot starts initialization");
  pwm.begin();
  pwm.setPWMFreq(60);  // Set frequency to 50 Hz for servos
  delay(10);

  pinMode(motorPinA1, OUTPUT);
  pinMode(motorPinA2, OUTPUT);
  pinMode(motorPinB1, OUTPUT);
  pinMode(motorPinB2, OUTPUT);
  pinMode(enA, OUTPUT);
  pinMode(enB, OUTPUT);


  analogWrite(enA, 60);
  analogWrite(enB, 50);



  // for (int i = 0; i < numChannels; i++) {
  //   pinMode(channelPins[i], INPUT);
  // }
  //initialize default parameter
  
  legion.set_site(0, x_default - x_offset, y_start + y_step, z_boot);
  legion.set_site(1, x_default - x_offset, y_start + y_step, z_boot);
  legion.set_site(2, x_default + x_offset, y_start, z_boot);
  legion.set_site(3, x_default + x_offset, y_start, z_boot);
  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      site_now[i][j] = site_expect[i][j];
    }
  }
  //start servo service
  FlexiTimer2::set(20, servo_service);
  FlexiTimer2::start();
  Serial.println("Servo service started");
  Serial.println("Servos initialized");
  Serial.println("Robot initialization Complete");
  
  legion.stand();
  //legion.climb();
}

int angleToPWM(float angle) {
  return map(angle, 0, 180, SERVOMIN, SERVOMAX);
}

// Set servo angles using PCA9685
void setServoAngle(int channel, float angle) {
  int pwmValue = angleToPWM(angle);
  pwm.setPWM(channel, 0, pwmValue);
}


void loop() {
  // RC_Control();
  // servicetest();
  gaits(cmd);
  //legion.climb();

}

// void serialEvent() {
//   char tmp = -1;
//   boolean taken = false;  // is steps already taken?

//   while (Serial.available()) {
//     state = !state;       // inverse state
//     tmp = Serial.read();  //Serial.println(cmd);
//     taken = gaits(tmp);   // steps taken?

//     if (taken)  // basically repeat the previous serial command
//       cmd = tmp;
//   }
// }

void serialEvent() {

  String command = "";
  while (Serial.available()) {
    state = !state;  // inverse state
    char c = (char)Serial.read();
    if (c == '\n') {  // End of command
      break;
    }
    command += c;
  }
  command.trim();  // Remove extra whitespace or newlines
  Serial.println(command);
  // Handle movement commands
  if (command == "f") cmd = FORWARD;
  else if (command == "b") cmd = BACKWARD;
  else if (command == "l") cmd = LEFT;
  else if (command == "r") cmd = RIGHT;
  else if (command == "s") cmd = STOP;
  else if (command == "w") cmd = MOVEFORWARD;
  else if (command == "x") cmd = MOVEBACKWARD;
  else if (command == "a") cmd = MOVELEFT;
  else if (command == "d") cmd = MOVERIGHT;
  else if (command == "h") cmd = DRIVEMODE;
  else if (command == "u"){
    legion.adjustheight(true);
  }
  else if (command == "v"){
    legion.adjustheight(false);
  }


  else if (command.startsWith("xd,")) {
    int value = command.substring(3).toInt();  // Extract value
    //Serial.println(command);
    legion.adjust('x', value);
  } else if (command.startsWith("yd,")) {
    int value = command.substring(3).toInt();  // Extract value
    legion.adjust('y', value);
    //Serial.println(command);
  } else if (command.startsWith("zd,")) {
    int value = command.substring(3).toInt();  // Extract value
    legion.adjust('z', value);
    //Serial.println(command);
  } else if (command.startsWith("ls,")) {
    int value = command.substring(3).toInt();  // Extract value
    legion.adjust('l', value);
    //Serial.println(command);
  } else if (command.startsWith("ss,")) {
    int value = command.substring(3).toInt();  // Extract value
    analogWrite(enA, value);
    analogWrite(enB, value);
    // legion.adjust("ss", value);
    //Serial.println(command);
  }
}

// void serialEvent() {
//   while (Serial.available()) {
//     char tmp = -1;
//     boolean taken = false;  // is steps already taken?
//     state = !state;       // inverse state
//     tmp = Serial.read();  //Serial.println(cmd);
//     taken = gaits(tmp);   // steps taken?

//     if (taken)  // basically repeat the previous serial command
//       cmd = tmp;
//   }
// }


// Function to process received serial commands

boolean gaits(char cmd) {
  bool taken = true;
  switch (cmd) {
    case FORWARD:
      legion.step_forward(1);
      break;
    case BACKWARD:
      legion.step_back(1);
      break;
    case RIGHT:
      legion.turn_right(1);
      break;
    case LEFT:
      legion.turn_left(1);
      break;
    case STOP:
      legion.stand();
      legion.MS();
      break;
    case SIT:
      legion.sit();
      break;
    case MOVEFORWARD:
      legion.MF();
      break;
    case MOVEBACKWARD:
      legion.MB();
      break;
    case MOVELEFT:
      legion.ML();
      break;
    case MOVERIGHT:
      legion.MR();
      break;
    case HANDWAVE:
      legion.hand_wave(3);
      break;
    case HANDSHAKE:
      legion.hand_shake(3);
      break;
    case HEIGHTUP:
      legion.adjustheight(true);
      //cmd=STOP;
      taken = false;
      break;
    case HEIGHTDOWN:
      legion.adjustheight(false);
      //cmd=STOP;
      taken = false;
      break;
    case DRIVEMODE:
      legion.drivemode();
      //cmd=STOP;
      taken = false;
      break;
    // case HEADUP:
    //   legion.head_up(3);
    //   break;
    // case HEADDOWN:
    //   legion.head_down(3);
    //   break;
    // case BODYRIGHT:
    //   legion.body_right(15);
    //   break;
    // case BODYLEFT:
    //   legion.body_left(15);
    //   break;
    // case DANCE:
    //   legion.body_dance(5);
    //   break;

    default:
      taken = false;
  }
  return taken;
}



/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/
void servo_service(void) {
  sei();
  static float alpha, beta, gamma;

  for (int i = 0; i < 4; i++) {
    for (int j = 0; j < 3; j++) {
      if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
        site_now[i][j] += temp_speed[i][j];
      else
        site_now[i][j] = site_expect[i][j];
    }

    cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
    polar_to_servo(i, alpha, beta, gamma);
  }

  rest_counter++;
}


void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z) {
  //calculate w-z degree
  float v, w;
  w = (x >= 0 ? 1 : -1) * (sqrt(pow(x, 2) + pow(y, 2)));
  v = w - length_c;
  alpha = atan2(z, v) + acos((pow(length_a, 2) - pow(length_b, 2) + pow(v, 2) + pow(z, 2)) / 2 / length_a / sqrt(pow(v, 2) + pow(z, 2)));
  beta = acos((pow(length_a, 2) + pow(length_b, 2) - pow(v, 2) - pow(z, 2)) / 2 / length_a / length_b);
  //calculate x-y-z degree
  gamma = (w >= 0) ? atan2(y, x) : atan2(-y, -x);
  //trans degree pi->180
  alpha = alpha / pi * 180;
  beta = beta / pi * 180;
  gamma = gamma / pi * 180;
}

/*
  - trans site from polar to microservos
  - mathematical model map to fact
  - the errors saved in eeprom will be add
   ---------------------------------------------------------------------------*/
void polar_to_servo(int leg, float alpha, float beta, float gamma) {
  if (leg == 0) {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  } else if (leg == 1) {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  } else if (leg == 2) {
    alpha += 90;
    beta = 180 - beta;
    gamma = 90 - gamma;
  } else if (leg == 3) {
    alpha = 90 - alpha;
    beta = beta;
    gamma += 90;
  }

  setServoAngle(servo_channels[leg][0], alpha);
  setServoAngle(servo_channels[leg][1], beta);
  setServoAngle(servo_channels[leg][2], gamma);
}

void RC_Control() {
  pulseWidths[0] = pulseIn(channelPins[0], HIGH, TIME_INTERVAL);
  pulseWidths[1] = pulseIn(channelPins[1], HIGH, TIME_INTERVAL);
  pulseWidths[2] = pulseIn(channelPins[2], HIGH, TIME_INTERVAL);
  pulseWidths[3] = pulseIn(channelPins[3], HIGH, TIME_INTERVAL);
  pulseWidths[4] = pulseIn(channelPins[4], HIGH, TIME_INTERVAL);
  // pulseWidths[5] = pulseIn(channelPins[5], HIGH, TIME_INTERVAL);

  // Serial.println(pulseWidths[0]);
  // Serial.println(pulseWidths[1]);

  channelValues[0] = (pulseWidths[0] > 1500) ? 1 : (pulseWidths[0] < 1100) ? -1
                                                                           : 0;
  channelValues[1] = (pulseWidths[1] > 1500) ? 1 : (pulseWidths[1] < 1100) ? -1
                                                                           : 0;
  // channelValues[2] = (pulseWidths[2] > 1500) ? 1 : (pulseWidths[2] < 1100) ? -1 : 0;
  // channelValues[3] = (pulseWidths[3] > 1500) ? 1 : (pulseWidths[3] < 1100) ? -1 : 0;
  // channelValues[4] = (pulseWidths[4] > 1500) ? 1 : (pulseWidths[4] < 1100) ? -1: 0;
  // channelValues[5] = (pulseWidths[5] > 1500) ? 1 : (pulseWidths[5] < 1100) ? -1 : 0;

  // // Initialize tmp2 to STAND by default
  boolean taken2 = false;  // is steps already taken?
  char tmp2 = -1;

  // // Determine command based on channel values
  if (channelValues[0] != 0) {
    if (channelValues[0] == -1) {
      tmp2 = LEFT;
    } else if (channelValues[0] == 1) {
      tmp2 = RIGHT;
    }
  } else if (channelValues[1] != 0) {
    if (channelValues[1] == -1) {
      tmp2 = BACKWARD;
    } else if (channelValues[1] == 1) {
      tmp2 = FORWARD;
    }
  } else {
    tmp2 = STOP;  // Default to STAND if no valid channel values
  }

  // heightValue = map(pulseWidths[2], 1000, 2000, 0, 100);  // Assuming 1000 to 2000 as pulse width range
  // heightValue = constrain(heightValue, 0, 100);  // Ensure the value stays within 0-100

  int Speed = map(pulseWidths[4], 1000, 2000, 300, 1500);  // Assuming 1000 to 2000 as pulse width range
  Speed = constrain(Speed, 300, 1500);                     // Ensure the value stays within 0-100  // Call adjustHeight with mapped value

  taken2 = gaits(tmp2);
  // // Update command and execute gait if changed

  if (taken2) {
    cmd = tmp2;  // Update cmd if the command was valid
  }
  delay(100);
}

void servicetest() {
  Serial.println("Stand");
  legion.stand();
  delay(2000);
  Serial.println("Step forward");
  legion.step_forward(3);
  delay(2000);
  Serial.println("Step back");
  legion.step_back(3);
  delay(2000);
  Serial.println("Turn left");
  legion.turn_left(3);
  delay(2000);
  Serial.println("Turn right");
  legion.turn_right(3);
  delay(2000);
  legion.hand_shake(3);
  delay(2000);
  legion.hand_wave(3);
  delay(2000);
  // legion.body_dance(3);
  // delay(2000);
  legion.sit();
  delay(2000);
  legion.stand();
  delay(2000);
  legion.head_up(20);
  delay(1000);
  legion.step_forward(3);
  delay(1000);
  legion.head_down(20);
  delay(1000);
  legion.head_down(20);
  delay(2000);
  legion.step_back(3);
  delay(2000);
  legion.sit();
  delay(2000);
  check_service = true;
}