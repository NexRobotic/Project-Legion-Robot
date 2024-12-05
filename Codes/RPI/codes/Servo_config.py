from adafruit_pca9685 import PCA9685
from board import SCL, SDA
import busio
import time

# Initialize I2C and PCA9685
i2c = busio.I2C(SCL, SDA)
pca = PCA9685(i2c)
pca.frequency = 60  # Set frequency to 50Hz for servos

# Servo channels for the 12 servos
servo_channels = list(range(15))  # Channels 0-16

# Define servo limits
SERVOMIN = 150  # Minimum pulse length count (from Arduino code)
SERVOMAX = 600  # Maximum pulse length count (from Arduino code)

# Helper function to set servo angle
def set_servo_angle(channel, angle):
    """
    Set a servo to a specific angle (0-180 degrees).
    :param channel: PCA9685 channel (0-11)
    :param angle: Desired angle (0-180 degrees)
    """
    pulse = int(SERVOMIN + (angle / 180.0) * (SERVOMAX - SERVOMIN))
    duty_cycle = int((pulse / 4096.0) * 65536)  # Convert to 16-bit duty cycle
    pca.channels[channel].duty_cycle = duty_cycle

# Main program to move all 12 servos to 90 degrees
try:
    print("Setting all servos to 90 degrees...")
    for channel in servo_channels:
        set_servo_angle(channel, 90)  # Set to 90 degrees
        time.sleep(0.1)  # Small delay to allow servo movement

    print("All servos are set to 90 degrees.")
    time.sleep(5)  # Keep the servos in position for 5 seconds
finally:
    # Deinitialize PCA9685 to free up resources
    pca.deinit()
    print("Program finished.")
