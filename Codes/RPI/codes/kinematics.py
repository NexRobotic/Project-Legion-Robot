import math
import time
import threading
from adafruit_servokit import ServoKit

class RobotKinematics:
    def __init__(self):
        # Initialize the PCA servo driver (16 channels)
        self.kit = ServoKit(channels=16)
        self.setup_constants()
        self.setup_servos()
        self.move_speed = 0.0  # Movement speed
        threading.Thread(target=self.servo_service, daemon=True).start()

    def setup_constants(self):
        # Robot dimensions and initial configurations
        self.length_a = 55.0
        self.length_b = 77.5
        self.length_c = 27.5
        self.length_side = 71.0
        self.z_absolute = -28.0

        self.z_default = -50.0
        self.z_up = -30.0
        self.z_boot = self.z_absolute
        self.x_default = 62.0
        self.x_offset = 0.0
        self.y_start = 0.0
        self.y_step = 40.0
        self.y_default = self.x_default

        self.KEEP = 255
        self.pi = math.pi

        # original Movement speed and parameters
        #self.speed_multiple = 1.0
        #self.spot_turn_speed = 4.0
        #self.leg_move_speed = 8.0
        #self.body_move_speed = 3.0
        #self.stand_seat_speed = 1.0
        
        # Movement speed and parameters
        self.speed_multiple = 1.0
        self.spot_turn_speed = 5.0
        self.leg_move_speed = 15.0
        self.body_move_speed = 5.0
        self.stand_seat_speed = 1.0

        # Calculations for turning
        temp_a = math.sqrt((2 * self.x_default + self.length_side) ** 2 + self.y_step ** 2)
        temp_b = 2 * (self.y_start + self.y_step) + self.length_side
        temp_c = math.sqrt((2 * self.x_default + self.length_side) ** 2 + (2 * self.y_start + self.y_step + self.length_side) ** 2)
        temp_alpha = math.acos((temp_a**2 + temp_b**2 - temp_c**2) / (2 * temp_a * temp_b))

        self.turn_x1 = (temp_a - self.length_side) / 2
        self.turn_y1 = self.y_start + self.y_step / 2
        self.turn_x0 = self.turn_x1 - temp_b * math.cos(temp_alpha)
        self.turn_y0 = temp_b * math.sin(temp_alpha) - self.turn_y1 - self.length_side

        # Arrays for coordinates and movement
        self.site_now = [[0.0, 0.0, 0.0] for _ in range(4)]
        self.site_expect = [[0.0, 0.0, 0.0] for _ in range(4)]
        self.temp_speed = [[0.0, 0.0, 0.0] for _ in range(4)]

        # Servo pin mappings
        self.servo_pin = [[0, 1, 2], [4, 5, 6], [8, 9, 10], [12, 13, 14]]

    def setup_servos(self):
        # Initialize servos for each leg
        self.servo = [[self.kit.servo[self.servo_pin[i][j]] for j in range(3)] for i in range(4)]

    def setup(self):
        # Robot initialization
        print("Robot starts initialization")
        self.set_site(0, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_boot)
        self.set_site(1, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_boot)
        self.set_site(2, self.x_default + self.x_offset, self.y_start, self.z_boot)
        self.set_site(3, self.x_default + self.x_offset, self.y_start, self.z_boot)

        for leg in range(4):
            for axis in range(3):
                self.site_now[leg][axis] = self.site_expect[leg][axis]

        # Initialize servos to neutral position
        for i in range(4):
            for j in range(3):
                self.servo[i][j].angle = 90
                time.sleep(0.1)

        print("Servos initialized")
        print("Robot initialization complete")

    def set_site(self, leg, x, y, z):
        # Set target position for a leg
        dx = (x - self.site_now[leg][0]) if x != self.KEEP else 0
        dy = (y - self.site_now[leg][1]) if y != self.KEEP else 0
        dz = (z - self.site_now[leg][2]) if z != self.KEEP else 0

        distance = math.sqrt(dx**2 + dy**2 + dz**2)
        if distance != 0:
            self.temp_speed[leg][0] = dx / distance * self.move_speed * self.speed_multiple
            self.temp_speed[leg][1] = dy / distance * self.move_speed * self.speed_multiple
            self.temp_speed[leg][2] = dz / distance * self.move_speed * self.speed_multiple

        if x != self.KEEP:
            self.site_expect[leg][0] = x
        if y != self.KEEP:
            self.site_expect[leg][1] = y
        if z != self.KEEP:
            self.site_expect[leg][2] = z

    def wait_reach(self, leg):
        # Wait for a leg to reach its target
        while True:
            if all(self.site_now[leg][i] == self.site_expect[leg][i] for i in range(3)):
                break
            time.sleep(0.01)

    def wait_all_reach(self):
        # Wait for all legs to reach their targets
        for leg in range(4):
            self.wait_reach(leg)

    def cartesian_to_polar(self, x, y, z):
        # Convert Cartesian coordinates to polar angles
        v = math.sqrt(x**2 + y**2) - self.length_c
        alpha = math.atan2(z, v) + math.acos(
            (self.length_a**2 - self.length_b**2 + v**2 + z**2) / (2 * self.length_a * math.sqrt(v**2 + z**2))
        )
        beta = math.acos((self.length_a**2 + self.length_b**2 - v**2 - z**2) / (2 * self.length_a * self.length_b))
        gamma = math.atan2(y, x)

        return math.degrees(alpha), math.degrees(beta), math.degrees(gamma)

    def polar_to_servo(self, leg, alpha, beta, gamma):
        # Adjust angles based on servo orientation
        if leg == 0:
            alpha = 90 - alpha
            beta = beta
            gamma += 90
        elif leg in [1, 2]:
            alpha += 90
            beta = 180 - beta
            gamma = 90 - gamma
        elif leg == 3:
            alpha = 90 - alpha
            beta = beta
            gamma += 90

        # Clamp angles to valid servo range
        self.servo[leg][0].angle = min(max(alpha, 0), 180)
        self.servo[leg][1].angle = min(max(beta, 0), 180)
        self.servo[leg][2].angle = min(max(gamma, 0), 180)

    def servo_service(self):
        # Update servos based on `site_now`
        while True:
            for leg in range(4):
                for axis in range(3):
                    if abs(self.site_now[leg][axis] - self.site_expect[leg][axis]) > abs(self.temp_speed[leg][axis]):
                        self.site_now[leg][axis] += self.temp_speed[leg][axis]
                    else:
                        self.site_now[leg][axis] = self.site_expect[leg][axis]

                alpha, beta, gamma = self.cartesian_to_polar(
                    self.site_now[leg][0], self.site_now[leg][1], self.site_now[leg][2]
                )
                self.polar_to_servo(leg, alpha, beta, gamma)
            time.sleep(0.02)

    def stand(self):
        # Stand the robot up
        self.move_speed = self.stand_seat_speed
        for leg in range(4):
            self.set_site(leg, self.KEEP, self.KEEP, self.z_default)
        self.wait_all_reach()

    def sit(self):
        # Sit the robot down
        self.move_speed = self.stand_seat_speed
        for leg in range(4):
            self.set_site(leg, self.KEEP, self.KEEP, self.z_boot)
        self.wait_all_reach()
        
    def step_forward(self, steps):
        self.move_speed = self.leg_move_speed
        while steps > 0:
            steps -= 1
            if self.site_now[2][1] == self.y_start:
                # Leg 2 and 1 move
                self.set_site(2, self.x_default + self.x_offset, self.y_start, self.z_up)
                self.wait_all_reach()
                self.set_site(2, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_up)
                self.wait_all_reach()
                self.set_site(2, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_default)
                self.wait_all_reach()

                self.move_speed = self.body_move_speed

                self.set_site(0, self.x_default + self.x_offset, self.y_start, self.z_default)
                self.set_site(1, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_default)
                self.set_site(2, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.set_site(3, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.wait_all_reach()

                self.move_speed = self.leg_move_speed

                self.set_site(1, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_up)
                self.wait_all_reach()
                self.set_site(1, self.x_default + self.x_offset, self.y_start, self.z_up)
                self.wait_all_reach()
                self.set_site(1, self.x_default + self.x_offset, self.y_start, self.z_default)
                self.wait_all_reach()
            else:
                # Leg 0 and 3 move
                self.set_site(0, self.x_default + self.x_offset, self.y_start, self.z_up)
                self.wait_all_reach()
                self.set_site(0, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_up)
                self.wait_all_reach()
                self.set_site(0, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_default)
                self.wait_all_reach()

                self.move_speed = self.body_move_speed

                self.set_site(0, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.set_site(1, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.set_site(2, self.x_default + self.x_offset, self.y_start, self.z_default)
                self.set_site(3, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_default)
                self.wait_all_reach()

                self.move_speed = self.leg_move_speed

                self.set_site(3, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_up)
                self.wait_all_reach()
                self.set_site(3, self.x_default + self.x_offset, self.y_start, self.z_up)
                self.wait_all_reach()
                self.set_site(3, self.x_default + self.x_offset, self.y_start, self.z_default)
                self.wait_all_reach()


    def step_back(self, steps):
        self.move_speed = self.leg_move_speed

        while steps > 0:
            steps -= 1
            if self.site_now[3][1] == self.y_start:
                # Move legs 3 and 0 backward
                self.set_site(3, self.x_default + self.x_offset, self.y_start, self.z_up)  # Lift leg 3
                self.wait_all_reach()
                self.set_site(3, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_up)  # Move backward
                self.wait_all_reach()
                self.set_site(3, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_default)  # Place leg 3 down
                self.wait_all_reach()

                self.move_speed = self.body_move_speed

                # Adjust body
                self.set_site(0, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_default)
                self.set_site(1, self.x_default + self.x_offset, self.y_start, self.z_default)
                self.set_site(2, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.set_site(3, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.wait_all_reach()

                self.move_speed = self.leg_move_speed

                self.set_site(0, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_up)  # Lift leg 0
                self.wait_all_reach()
                self.set_site(0, self.x_default + self.x_offset, self.y_start, self.z_up)  # Move backward
                self.wait_all_reach()
                self.set_site(0, self.x_default + self.x_offset, self.y_start, self.z_default)  # Place leg 0 down
                self.wait_all_reach()
            else:
                # Move legs 1 and 2 backward
                self.set_site(1, self.x_default + self.x_offset, self.y_start, self.z_up)  # Lift leg 1
                self.wait_all_reach()
                self.set_site(1, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_up)  # Move backward
                self.wait_all_reach()
                self.set_site(1, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_default)  # Place leg 1 down
                self.wait_all_reach()

                self.move_speed = self.body_move_speed

                # Adjust body
                self.set_site(0, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.set_site(1, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.set_site(2, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_default)
                self.set_site(3, self.x_default + self.x_offset, self.y_start, self.z_default)
                self.wait_all_reach()

                self.move_speed = self.leg_move_speed

                self.set_site(2, self.x_default + self.x_offset, self.y_start + 2 * self.y_step, self.z_up)  # Lift leg 2
                self.wait_all_reach()
                self.set_site(2, self.x_default + self.x_offset, self.y_start, self.z_up)  # Move backward
                self.wait_all_reach()
                self.set_site(2, self.x_default + self.x_offset, self.y_start, self.z_default)  # Place leg 2 down
                self.wait_all_reach()


    def turn_right(self, steps):
        self.move_speed = self.spot_turn_speed

        for _ in range(steps):
            if self.site_now[2][1] == self.y_start:
                # Move legs 2 and 0
                self.set_site(2, self.x_default + self.x_offset, self.y_start, self.z_up)  # Lift leg 2
                self.wait_all_reach()

                self.set_site(0, self.turn_x0 - self.x_offset, self.turn_y0, self.z_default)  # Adjust body
                self.set_site(1, self.turn_x1 - self.x_offset, self.turn_y1, self.z_default)
                self.set_site(2, self.turn_x0 + self.x_offset, self.turn_y0, self.z_up)  # Move leg 2
                self.set_site(3, self.turn_x1 + self.x_offset, self.turn_y1, self.z_default)
                self.wait_all_reach()

                self.set_site(2, self.turn_x0 + self.x_offset, self.turn_y0, self.z_default)  # Place leg 2 down
                self.wait_all_reach()

                self.set_site(0, self.turn_x0 + self.x_offset, self.turn_y0, self.z_default)  # Adjust body again
                self.set_site(1, self.turn_x1 + self.x_offset, self.turn_y1, self.z_default)
                self.set_site(2, self.turn_x0 - self.x_offset, self.turn_y0, self.z_default)
                self.set_site(3, self.turn_x1 - self.x_offset, self.turn_y1, self.z_default)
                self.wait_all_reach()

                self.set_site(0, self.turn_x0 + self.x_offset, self.turn_y0, self.z_up)  # Lift leg 0
                self.wait_all_reach()

                self.set_site(0, self.x_default + self.x_offset, self.y_start, self.z_up)  # Move leg 0 back
                self.set_site(1, self.x_default + self.x_offset, self.y_start, self.z_default)
                self.set_site(2, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.set_site(3, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.wait_all_reach()

                self.set_site(0, self.x_default + self.x_offset, self.y_start, self.z_default)  # Place leg 0 down
                self.wait_all_reach()
            else:
                # Move legs 1 and 3
                self.set_site(1, self.x_default + self.x_offset, self.y_start, self.z_up)  # Lift leg 1
                self.wait_all_reach()

                self.set_site(0, self.turn_x1 + self.x_offset, self.turn_y1, self.z_default)  # Adjust body
                self.set_site(1, self.turn_x0 + self.x_offset, self.turn_y0, self.z_up)  # Move leg 1
                self.set_site(2, self.turn_x1 - self.x_offset, self.turn_y1, self.z_default)
                self.set_site(3, self.turn_x0 - self.x_offset, self.turn_y0, self.z_default)
                self.wait_all_reach()

                self.set_site(1, self.turn_x0 + self.x_offset, self.turn_y0, self.z_default)  # Place leg 1 down
                self.wait_all_reach()

                self.set_site(0, self.turn_x1 - self.x_offset, self.turn_y1, self.z_default)  # Adjust body again
                self.set_site(1, self.turn_x0 - self.x_offset, self.turn_y0, self.z_default)
                self.set_site(2, self.turn_x1 + self.x_offset, self.turn_y1, self.z_default)
                self.set_site(3, self.turn_x0 + self.x_offset, self.turn_y0, self.z_default)
                self.wait_all_reach()

                self.set_site(3, self.turn_x0 + self.x_offset, self.turn_y0, self.z_up)  # Lift leg 3
                self.wait_all_reach()

                self.set_site(0, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)  # Adjust body
                self.set_site(1, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.set_site(2, self.x_default + self.x_offset, self.y_start, self.z_default)
                self.set_site(3, self.x_default + self.x_offset, self.y_start, self.z_up)  # Move leg 3 back
                self.wait_all_reach()

                self.set_site(3, self.x_default + self.x_offset, self.y_start, self.z_default)  # Place leg 3 down
                self.wait_all_reach()

    def turn_left(self, steps):
        self.move_speed = self.spot_turn_speed

        for _ in range(steps):
            if self.site_now[3][1] == self.y_start:
                # Move legs 3 and 1
                self.set_site(3, self.x_default + self.x_offset, self.y_start, self.z_up)  # Lift leg 3
                self.wait_all_reach()

                self.set_site(0, self.turn_x1 - self.x_offset, self.turn_y1, self.z_default)  # Adjust body
                self.set_site(1, self.turn_x0 - self.x_offset, self.turn_y0, self.z_default)
                self.set_site(2, self.turn_x1 + self.x_offset, self.turn_y1, self.z_default)
                self.set_site(3, self.turn_x0 + self.x_offset, self.turn_y0, self.z_up)  # Move leg 3
                self.wait_all_reach()

                self.set_site(3, self.turn_x0 + self.x_offset, self.turn_y0, self.z_default)  # Place leg 3 down
                self.wait_all_reach()

                self.set_site(0, self.turn_x1 + self.x_offset, self.turn_y1, self.z_default)  # Adjust body again
                self.set_site(1, self.turn_x0 + self.x_offset, self.turn_y0, self.z_default)
                self.set_site(2, self.turn_x1 - self.x_offset, self.turn_y1, self.z_default)
                self.set_site(3, self.turn_x0 - self.x_offset, self.turn_y0, self.z_default)
                self.wait_all_reach()

                self.set_site(1, self.turn_x0 + self.x_offset, self.turn_y0, self.z_up)  # Lift leg 1
                self.wait_all_reach()

                self.set_site(0, self.x_default + self.x_offset, self.y_start, self.z_default)  # Adjust body
                self.set_site(1, self.x_default + self.x_offset, self.y_start, self.z_up)  # Move leg 1
                self.set_site(2, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.set_site(3, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.wait_all_reach()

                self.set_site(1, self.x_default + self.x_offset, self.y_start, self.z_default)  # Place leg 1 down
                self.wait_all_reach()
            else:
                # Move legs 0 and 2
                self.set_site(0, self.x_default + self.x_offset, self.y_start, self.z_up)  # Lift leg 0
                self.wait_all_reach()

                self.set_site(0, self.turn_x0 + self.x_offset, self.turn_y0, self.z_up)  # Move leg 0
                self.set_site(1, self.turn_x1 + self.x_offset, self.turn_y1, self.z_default)
                self.set_site(2, self.turn_x0 - self.x_offset, self.turn_y0, self.z_default)
                self.set_site(3, self.turn_x1 - self.x_offset, self.turn_y1, self.z_default)
                self.wait_all_reach()

                self.set_site(0, self.turn_x0 + self.x_offset, self.turn_y0, self.z_default)  # Place leg 0 down
                self.wait_all_reach()

                self.set_site(0, self.turn_x0 - self.x_offset, self.turn_y0, self.z_default)  # Adjust body
                self.set_site(1, self.turn_x1 - self.x_offset, self.turn_y1, self.z_default)
                self.set_site(2, self.turn_x0 + self.x_offset, self.turn_y0, self.z_default)
                self.set_site(3, self.turn_x1 + self.x_offset, self.turn_y1, self.z_default)
                self.wait_all_reach()

                self.set_site(2, self.turn_x0 + self.x_offset, self.turn_y0, self.z_up)  # Lift leg 2
                self.wait_all_reach()

                self.set_site(0, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)  # Adjust body
                self.set_site(1, self.x_default - self.x_offset, self.y_start + self.y_step, self.z_default)
                self.set_site(2, self.x_default + self.x_offset, self.y_start, self.z_up)  # Move leg 2
                self.set_site(3, self.x_default + self.x_offset, self.y_start, self.z_default)
                self.wait_all_reach()

                self.set_site(2, self.x_default + self.x_offset, self.y_start, self.z_default)  # Place leg 2 down
                self.wait_all_reach()

    def body_left(self, i):
        self.set_site(0, self.site_now[0][0] + i, self.KEEP, self.KEEP)
        self.set_site(1, self.site_now[1][0] + i, self.KEEP, self.KEEP)
        self.set_site(2, self.site_now[2][0] - i, self.KEEP, self.KEEP)
        self.set_site(3, self.site_now[3][0] - i, self.KEEP, self.KEEP)
        self.wait_all_reach()

    def body_right(self, i):
        self.set_site(0, self.site_now[0][0] - i, self.KEEP, self.KEEP)
        self.set_site(1, self.site_now[1][0] - i, self.KEEP, self.KEEP)
        self.set_site(2, self.site_now[2][0] + i, self.KEEP, self.KEEP)
        self.set_site(3, self.site_now[3][0] + i, self.KEEP, self.KEEP)
        self.wait_all_reach()

    def hand_wave(self, i):
        self.move_speed = 1
        if self.site_now[3][1] == self.y_start:
            self.body_right(15)
            x_tmp, y_tmp, z_tmp = self.site_now[2]
            self.move_speed = self.body_move_speed
            for _ in range(i):
                self.set_site(2, self.turn_x1, self.turn_y1, 50)
                self.wait_all_reach()
                self.set_site(2, self.turn_x0, self.turn_y0, 50)
                self.wait_all_reach()
            self.set_site(2, x_tmp, y_tmp, z_tmp)
            self.wait_all_reach()
            self.move_speed = 1
            self.body_left(15)
        else:
            self.body_left(15)
            x_tmp, y_tmp, z_tmp = self.site_now[0]
            self.move_speed = self.body_move_speed
            for _ in range(i):
                self.set_site(0, self.turn_x1, self.turn_y1, 50)
                self.wait_all_reach()
                self.set_site(0, self.turn_x0, self.turn_y0, 50)
                self.wait_all_reach()
            self.set_site(0, x_tmp, y_tmp, z_tmp)
            self.wait_all_reach()
            self.move_speed = 1
            self.body_right(15)

    def hand_shake(self, i):
        self.move_speed = 1
        if self.site_now[3][1] == self.y_start:
            self.body_right(15)
            x_tmp, y_tmp, z_tmp = self.site_now[2]
            self.move_speed = self.body_move_speed
            for _ in range(i):
                self.set_site(2, self.x_default - 30, self.y_start + 2 * self.y_step, 55)
                self.wait_all_reach()
                self.set_site(2, self.x_default - 30, self.y_start + 2 * self.y_step, 10)
                self.wait_all_reach()
            self.set_site(2, x_tmp, y_tmp, z_tmp)
            self.wait_all_reach()
            self.move_speed = 1
            self.body_left(15)
        else:
            self.body_left(15)
            x_tmp, y_tmp, z_tmp = self.site_now[0]
            self.move_speed = self.body_move_speed
            for _ in range(i):
                self.set_site(0, self.x_default - 30, self.y_start + 2 * self.y_step, 55)
                self.wait_all_reach()
                self.set_site(0, self.x_default - 30, self.y_start + 2 * self.y_step, 10)
                self.wait_all_reach()
            self.set_site(0, x_tmp, y_tmp, z_tmp)
            self.wait_all_reach()
            self.move_speed = 1
            self.body_right(15)

    def head_up(self, i):
        self.set_site(0, self.KEEP, self.KEEP, self.site_now[0][2] - i)
        self.set_site(1, self.KEEP, self.KEEP, self.site_now[1][2] + i)
        self.set_site(2, self.KEEP, self.KEEP, self.site_now[2][2] - i)
        self.set_site(3, self.KEEP, self.KEEP, self.site_now[3][2] + i)
        self.wait_all_reach()

    def head_down(self, i):
        self.set_site(0, self.KEEP, self.KEEP, self.site_now[0][2] + i)
        self.set_site(1, self.KEEP, self.KEEP, self.site_now[1][2] - i)
        self.set_site(2, self.KEEP, self.KEEP, self.site_now[2][2] + i)
        self.set_site(3, self.KEEP, self.KEEP, self.site_now[3][2] - i)
        self.wait_all_reach()

    def body_dance(self, i):
        self.body_dance_speed = 2
        self.sit()
        self.move_speed = 1
        for leg in range(4):
            self.set_site(leg, self.x_default, self.y_default, self.KEEP)
        self.wait_all_reach()

        # Lower body slightly
        for leg in range(4):
            self.set_site(leg, self.x_default, self.y_default, self.z_default - 20)
        self.wait_all_reach()

        self.move_speed = self.body_dance_speed
        self.head_up(30)
        for j in range(i):
            if j > i / 4:
                self.move_speed = self.body_dance_speed * 2
            if j > i / 2:
                self.move_speed = self.body_dance_speed * 3
            self.set_site(0, self.KEEP, self.y_default - 20, self.KEEP)
            self.set_site(1, self.KEEP, self.y_default + 20, self.KEEP)
            self.set_site(2, self.KEEP, self.y_default - 20, self.KEEP)
            self.set_site(3, self.KEEP, self.y_default + 20, self.KEEP)
            self.wait_all_reach()
            self.set_site(0, self.KEEP, self.y_default + 20, self.KEEP)
            self.set_site(1, self.KEEP, self.y_default - 20, self.KEEP)
            self.set_site(2, self.KEEP, self.y_default + 20, self.KEEP)
            self.set_site(3, self.KEEP, self.y_default - 20, self.KEEP)
            self.wait_all_reach()

        self.move_speed = self.body_dance_speed
        self.head_down(30)


# if __name__ == "__main__":
#     # Example usage
#     robot = RobotKinematics()
#     robot.setup()
#     robot.stand()
#     time.sleep(2)
#     robot.sit()

