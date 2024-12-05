from kinematics import RobotKinematics
import time

def main():
    # Initialize the robot
    robot = RobotKinematics()
    robot.setup()
    #time.sleep(2)

    # Sequence of actions
    print("Robot standing...")
    robot.stand()
    time.sleep(0.5)

    print("Robot stepping forward...")
    robot.step_forward(2)
    #ime.sleep(2)

    print("Robot stepping backward...")
    robot.step_back(2)
    #time.sleep(1)

    print("Robot turning right...")
    robot.turn_right(2)
    #time.sleep(1)

    print("Robot turning left...")
    robot.turn_left(2)
    #time.sleep(1)

    print("Robot moving body left...")
    robot.body_left(15)
    #time.sleep(1)

    print("Robot moving body right...")
    robot.body_right(15)
    #time.sleep(1)

    print("Robot hand waving...")
    robot.hand_wave(3)
    #time.sleep(1)

    print("Robot hand shaking...")
    robot.hand_shake(3)
    #time.sleep(1)

    print("Robot dancing...")
    robot.body_dance(5)
    #time.sleep(1)

    print("Robot sitting...")
    robot.sit()
    #time.sleep(1)
    
    print("Demo complete.")

if __name__ == "__main__":
    main()
