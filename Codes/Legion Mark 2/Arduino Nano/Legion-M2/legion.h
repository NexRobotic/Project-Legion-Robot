#ifndef LEGION_H
#define LEGION_H


#include <math.h> 

class Legion {
public:
  Legion();  // Constructor

  //Wheels Functions
  void MF();
  void MB();
  void MR();
  void ML();
  void MS();
  void drivemode();

  void adjust(char axis, int val);
  void adjustheight(bool flag);
  void setup();
  void loop();

  void stand();
  void sit();
  void step_forward(unsigned int step);
  void step_back(unsigned int step);
  void turn_left(unsigned int step);
  void turn_right(unsigned int step);
  void climb();

  void body_left(int i);
  void body_right(int i);

  void hand_wave(int i);
  void hand_shake(int i);

  void head_up(int i);
  void head_down(int i);

  void body_dance(int i);
  static void servo_service(void);
  static void servo_service_wrapper();
  static Legion *instance;


  void servo_attach();
  void servo_detach();
  void wait_all_reach();
  void wait_reach(int leg);
  void set_site(int leg, float x, float y, float z);
  void cartesian_to_polar(volatile float &alpha, volatile float &beta, volatile float &gamma, volatile float x, volatile float y, volatile float z);
  void polar_to_servo(int leg, float alpha, float beta, float gamma);
  };

#endif
