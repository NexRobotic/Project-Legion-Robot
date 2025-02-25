#include "Legion.h"
#include "Arduino.h"
#include "Legion_config.h"

Legion::Legion() {
}


// Initialize the static member variable


// void Legion::servo_service(void)
// {
//   sei();
//   static float alpha, beta, gamma;

//   for (int i = 0; i < 4; i++)
//   {
//     for (int j = 0; j < 3; j++)
//     {
//       if (abs(site_now[i][j] - site_expect[i][j]) >= abs(temp_speed[i][j]))
//         site_now[i][j] += temp_speed[i][j];
//       else
//         site_now[i][j] = site_expect[i][j];
//     }

//     cartesian_to_polar(alpha, beta, gamma, site_now[i][0], site_now[i][1], site_now[i][2]);
//     polar_to_servo(i, alpha, beta, gamma);
//   }

//   rest_counter++;
// }


Legion* Legion::instance = nullptr;  // Initialize the static member

void Legion::servo_service_wrapper() {
  instance->servo_service();
}

// Wheels Code
void Legion::ML() {
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB1, HIGH);
  digitalWrite(motorPinB2, LOW);
}

void Legion::MR() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinA2, HIGH);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinB2, HIGH);
}

void Legion::MF() {
  digitalWrite(motorPinA1, HIGH);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinB2, HIGH);
}

void Legion::MB() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinA2, HIGH);
  digitalWrite(motorPinB1, HIGH);
  digitalWrite(motorPinB2, LOW);
}

void Legion::MS() {
  digitalWrite(motorPinA1, LOW);
  digitalWrite(motorPinA2, LOW);
  digitalWrite(motorPinB1, LOW);
  digitalWrite(motorPinB2, LOW);
}

void Legion::drivemode() {
  set_site(0, KEEP, KEEP, z_default);
  set_site(0, KEEP, y_start, KEEP);
  set_site(0, x_default, KEEP, KEEP);

  set_site(1, KEEP, KEEP, z_default);
  set_site(1, KEEP, y_start, KEEP);
  set_site(1, x_default, KEEP, KEEP);

  set_site(2, KEEP, KEEP, z_default);
  set_site(2, KEEP, y_start, KEEP);
  set_site(2, x_default, KEEP, KEEP);

  set_site(3, KEEP, KEEP, z_default);
  set_site(3, KEEP, y_start, KEEP);
  set_site(3, x_default, KEEP, KEEP);
}

void Legion ::climb() {
  float localz = z_default/2;
  move_speed = leg_move_speed;

  if (site_now[2][1] == y_start) {
    //leg 2&1 move
    set_site(2, x_default + x_offset, y_start, z_up);
    wait_all_reach();
    set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
    wait_all_reach();
    set_site(2, x_default + x_offset, y_start + 2 * y_step, localz);
    wait_all_reach();

    Serial.println("1");
    delay(1000);

    move_speed = body_move_speed;

    set_site(0, x_default + x_offset, y_start, z_default);
    set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
    set_site(2, x_default - x_offset, y_start + y_step, localz);
    set_site(3, x_default - x_offset, y_start + y_step, z_default);
    wait_all_reach();

    Serial.println("2");
    delay(1000);

    move_speed = leg_move_speed;

    set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
    wait_all_reach();
    set_site(1, x_default + x_offset, y_start, z_up);
    wait_all_reach();
    set_site(1, x_default + x_offset, y_start, z_default);
    wait_all_reach();

    Serial.println("3");
    delay(1000);
  } else {
    //leg 0&3 move
    set_site(0, x_default + x_offset, y_start, z_up);
    wait_all_reach();
    set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
    wait_all_reach();
    set_site(0, x_default + x_offset, y_start + 2 * y_step, localz);
    wait_all_reach();

    Serial.println("4");
    delay(1000);

    move_speed = body_move_speed;

    set_site(0, x_default - x_offset, y_start + y_step, localz);
    set_site(1, x_default - x_offset, y_start + y_step, z_default);
    set_site(2, x_default + x_offset, y_start, localz);
    set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
    wait_all_reach();

    Serial.println("5");
    delay(1000);


    move_speed = leg_move_speed;

    set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
    wait_all_reach();
    set_site(3, x_default + x_offset, y_start, z_up);
    wait_all_reach();
    set_site(3, x_default + x_offset, y_start, z_default);
    wait_all_reach();
    Serial.println("6");
    delay(1000);
  }
}
// Wheels Code



/*
  - sit
  - blocking function
   ---------------------------------------------------------------------------*/
void Legion::sit(void) {
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, KEEP, KEEP, z_boot);
  }
  wait_all_reach();
}

/*
  - stand
  - blocking function
   ---------------------------------------------------------------------------*/
void Legion::stand(void) {
  move_speed = stand_seat_speed;
  for (int leg = 0; leg < 4; leg++) {
    set_site(leg, KEEP, KEEP, z_default);
  }
  wait_all_reach();
}


/*
  - spot turn to left
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void Legion::adjust(char axis, int val) {
  Serial.println(axis);
  Serial.println(val);
  switch (axis) {
    case 'x':  // Handle x axis
      x_default = val;
      break;
    case 'y':  // Handle y axis
      y_default = val;
      break;
    case 'z':  // Handle z axis
      z_default = val;
      break;
    case 'l':  // Handle leg speed
      leg_move_speed = val;
      break;
    default:
      return;
  }
  Serial.println(x_default);
  Serial.println(y_default);
  Serial.println(z_default);

  // Update all sites
  set_site(0, x_default, y_start, z_default);
  set_site(1, x_default, y_start, z_default);
  set_site(2, x_default, y_start, z_default);
  set_site(3, x_default, y_start, z_default);

  // Wait for all to reach the destination
  wait_all_reach();
}

void Legion::adjustheight(bool flag) {
  if (flag) {
    z_default = z_default - 5;
    //y_default = y_default -5;
    x_default = x_default - 3;
    //z_up = z_up+5;
    y_step = y_step + 1;
  } else {
    z_default = z_default + 5;
    //y_default = y_default +5;
    x_default = x_default + 3;
    y_step = y_step - 1;
    //z_up = z_up-5;
  }
  // Update all sites
  set_site(0, KEEP, KEEP, z_default);
  set_site(1, KEEP, KEEP, z_default);
  set_site(2, KEEP, KEEP, z_default);
  set_site(3, KEEP, KEEP, z_default);

  // Wait for all to reach the destination
  wait_all_reach();
}


void Legion::turn_left(unsigned int step) {
  move_speed = spot_turn_speed;
  while (step-- > 0) {
    if (site_now[3][1] == y_start) {
      //leg 3&1 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start, z_up);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    } else {
      //leg 0&2 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_up);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - spot turn to right
  - blocking function
  - parameter step steps wanted to turn
   ---------------------------------------------------------------------------*/
void Legion::turn_right(unsigned int step) {
  move_speed = spot_turn_speed;
  while (step-- > 0) {
    if (site_now[2][1] == y_start) {
      //leg 2&0 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x0 - x_offset, turn_y0, z_default);
      set_site(1, turn_x1 - x_offset, turn_y1, z_default);
      set_site(2, turn_x0 + x_offset, turn_y0, z_up);
      set_site(3, turn_x1 + x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(2, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_default);
      set_site(1, turn_x1 + x_offset, turn_y1, z_default);
      set_site(2, turn_x0 - x_offset, turn_y0, z_default);
      set_site(3, turn_x1 - x_offset, turn_y1, z_default);
      wait_all_reach();

      set_site(0, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_up);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    } else {
      //leg 1&3 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(0, turn_x1 + x_offset, turn_y1, z_default);
      set_site(1, turn_x0 + x_offset, turn_y0, z_up);
      set_site(2, turn_x1 - x_offset, turn_y1, z_default);
      set_site(3, turn_x0 - x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(1, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(0, turn_x1 - x_offset, turn_y1, z_default);
      set_site(1, turn_x0 - x_offset, turn_y0, z_default);
      set_site(2, turn_x1 + x_offset, turn_y1, z_default);
      set_site(3, turn_x0 + x_offset, turn_y0, z_default);
      wait_all_reach();

      set_site(3, turn_x0 + x_offset, turn_y0, z_up);
      wait_all_reach();

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();

      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go forward
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void Legion::step_forward(unsigned int step) {
  move_speed = leg_move_speed;
  while (step-- > 0) {
    if (site_now[2][1] == y_start) {
      //leg 2&1 move
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start, z_default);
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    } else {
      //leg 0&3 move
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start, z_default);
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

/*
  - go back
  - blocking function
  - parameter step steps wanted to go
   ---------------------------------------------------------------------------*/
void Legion::step_back(unsigned int step) {
  move_speed = leg_move_speed;
  while (step-- > 0) {
    if (site_now[3][1] == y_start) {
      //leg 3&0 move
      set_site(3, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(3, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(1, x_default + x_offset, y_start, z_default);
      set_site(2, x_default - x_offset, y_start + y_step, z_default);
      set_site(3, x_default - x_offset, y_start + y_step, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(0, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(0, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    } else {
      //leg 1&2 move
      set_site(1, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(1, x_default + x_offset, y_start + 2 * y_step, z_default);
      wait_all_reach();

      move_speed = body_move_speed;

      set_site(0, x_default - x_offset, y_start + y_step, z_default);
      set_site(1, x_default - x_offset, y_start + y_step, z_default);
      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_default);
      set_site(3, x_default + x_offset, y_start, z_default);
      wait_all_reach();

      move_speed = leg_move_speed;

      set_site(2, x_default + x_offset, y_start + 2 * y_step, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_up);
      wait_all_reach();
      set_site(2, x_default + x_offset, y_start, z_default);
      wait_all_reach();
    }
  }
}

// add by RegisHsu

void Legion::body_left(int i) {
  set_site(0, site_now[0][0] + i, KEEP, KEEP);
  set_site(1, site_now[1][0] + i, KEEP, KEEP);
  set_site(2, site_now[2][0] - i, KEEP, KEEP);
  set_site(3, site_now[3][0] - i, KEEP, KEEP);
  wait_all_reach();
}

void Legion::body_right(int i) {
  set_site(0, site_now[0][0] - i, KEEP, KEEP);
  set_site(1, site_now[1][0] - i, KEEP, KEEP);
  set_site(2, site_now[2][0] + i, KEEP, KEEP);
  set_site(3, site_now[3][0] + i, KEEP, KEEP);
  wait_all_reach();
}

void Legion::hand_wave(int i) {
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start) {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++) {
      set_site(2, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(2, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  } else {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++) {
      set_site(0, turn_x1, turn_y1, 50);
      wait_all_reach();
      set_site(0, turn_x0, turn_y0, 50);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void Legion::hand_shake(int i) {
  float x_tmp;
  float y_tmp;
  float z_tmp;
  move_speed = 1;
  if (site_now[3][1] == y_start) {
    body_right(15);
    x_tmp = site_now[2][0];
    y_tmp = site_now[2][1];
    z_tmp = site_now[2][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++) {
      set_site(2, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(2, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(2, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_left(15);
  } else {
    body_left(15);
    x_tmp = site_now[0][0];
    y_tmp = site_now[0][1];
    z_tmp = site_now[0][2];
    move_speed = body_move_speed;
    for (int j = 0; j < i; j++) {
      set_site(0, x_default - 30, y_start + 2 * y_step, 55);
      wait_all_reach();
      set_site(0, x_default - 30, y_start + 2 * y_step, 10);
      wait_all_reach();
    }
    set_site(0, x_tmp, y_tmp, z_tmp);
    wait_all_reach();
    move_speed = 1;
    body_right(15);
  }
}

void Legion::head_up(int i) {
  set_site(0, KEEP, KEEP, site_now[0][2] - i);

  set_site(1, KEEP, KEEP, site_now[1][2] + i);

  set_site(2, KEEP, KEEP, site_now[2][2] - i);

  set_site(3, KEEP, KEEP, site_now[3][2] + i);

  wait_all_reach();
}

void Legion::head_down(int i) {
  set_site(0, KEEP, KEEP, site_now[0][2] + i);
  set_site(1, KEEP, KEEP, site_now[1][2] - i);
  set_site(2, KEEP, KEEP, site_now[2][2] + i);
  set_site(3, KEEP, KEEP, site_now[3][2] - i);
  wait_all_reach();
}

void Legion::body_dance(int i) {
  float x_tmp;
  float y_tmp;
  float z_tmp;
  float body_dance_speed = 2;
  sit();
  move_speed = 1;
  set_site(0, x_default, y_default, KEEP);
  set_site(1, x_default, y_default, KEEP);
  set_site(2, x_default, y_default, KEEP);
  set_site(3, x_default, y_default, KEEP);
  wait_all_reach();
  //stand();
  set_site(0, x_default, y_default, z_default - 20);
  set_site(1, x_default, y_default, z_default - 20);
  set_site(2, x_default, y_default, z_default - 20);
  set_site(3, x_default, y_default, z_default - 20);
  wait_all_reach();
  move_speed = body_dance_speed;
  head_up(30);
  for (int j = 0; j < i; j++) {
    if (j > i / 4)
      move_speed = body_dance_speed * 2;
    if (j > i / 2)
      move_speed = body_dance_speed * 3;
    set_site(0, KEEP, y_default - 20, KEEP);
    set_site(1, KEEP, y_default + 20, KEEP);
    set_site(2, KEEP, y_default - 20, KEEP);
    set_site(3, KEEP, y_default + 20, KEEP);
    wait_all_reach();
    set_site(0, KEEP, y_default + 20, KEEP);
    set_site(1, KEEP, y_default - 20, KEEP);
    set_site(2, KEEP, y_default + 20, KEEP);
    set_site(3, KEEP, y_default - 20, KEEP);
    wait_all_reach();
  }
  move_speed = body_dance_speed;
  head_down(30);
}


/*
  - microservos service /timer interrupt function/50Hz
  - when set site expected,this function move the end point to it in a straight line
  - temp_speed[4][3] should be set before set expect site,it make sure the end point
   move in a straight line,and decide move speed.
   ---------------------------------------------------------------------------*/


/*
  - set one of end points' expect site
  - this founction will set temp_speed[4][3] at same time
  - non - blocking function
   ---------------------------------------------------------------------------*/
void Legion::set_site(int leg, float x, float y, float z) {
  float length_x = 0, length_y = 0, length_z = 0;

  if (x != KEEP)
    length_x = x - site_now[leg][0];
  if (y != KEEP)
    length_y = y - site_now[leg][1];
  if (z != KEEP)
    length_z = z - site_now[leg][2];

  float length = sqrt(pow(length_x, 2) + pow(length_y, 2) + pow(length_z, 2));

  temp_speed[leg][0] = length_x / length * move_speed * speed_multiple;
  temp_speed[leg][1] = length_y / length * move_speed * speed_multiple;
  temp_speed[leg][2] = length_z / length * move_speed * speed_multiple;

  if (x != KEEP)
    site_expect[leg][0] = x;
  if (y != KEEP)
    site_expect[leg][1] = y;
  if (z != KEEP)
    site_expect[leg][2] = z;
}

/*
  - wait one of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void Legion::wait_reach(int leg) {
  // Serial.println("inside wait");

  while (1) {  // Ensure braces properly open here
    // Print current and expected positions
    // Serial.println("Current positions:");
    // Serial.println(site_now[leg][2]);
    // Serial.println(site_expect[leg][2]);

    // Check if the current position matches the expected position
    if (site_now[leg][0] == site_expect[leg][0] && site_now[leg][1] == site_expect[leg][1] && site_now[leg][2] == site_expect[leg][2]) {
      break;  // Exit the loop when all coordinates match
    }
  }
  // Serial.println("exit wait");
}

/*
  - wait all of end points move to expect site
  - blocking function
   ---------------------------------------------------------------------------*/
void Legion::wait_all_reach(void) {
  for (int i = 0; i < 4; i++)
    wait_reach(i);
}
