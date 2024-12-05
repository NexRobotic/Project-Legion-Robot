import cv2
import numpy as np
from robot_kinematics import RobotKinematics

class ObjectTracker:
    def __init__(self):
        self.robot = RobotKinematics()  # Initialize robot kinematics
        self.robot.setup()
        self.robot.stand()
        self.capture = cv2.VideoCapture(0)  # Use the first camera (can replace with a specific path)
        self.target_position = (320, 240)  # Assume center of the frame (for 640x480 resolution)
        self.frame_center = (320, 240)  # Center of a 640x480 frame
        self.threshold = 30  # Pixel threshold for considering the object aligned

    def process_frame(self, frame):
        """Process the frame to detect the object and get its position."""
        # Convert to HSV color space
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)

        # Define color range for tracking (e.g., red object)
        lower_color = np.array([0, 100, 100])  # Adjust for your target object
        upper_color = np.array([10, 255, 255])

        # Create a mask for the target color
        mask = cv2.inRange(hsv, lower_color, upper_color)

        # Find contours in the mask
        contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        if contours:
            # Find the largest contour
            largest_contour = max(contours, key=cv2.contourArea)
            if cv2.contourArea(largest_contour) > 500:  # Ignore small objects
                # Get the center of the object
                M = cv2.moments(largest_contour)
                if M["m00"] > 0:
                    cx = int(M["m10"] / M["m00"])
                    cy = int(M["m01"] / M["m00"])
                    return (cx, cy)

        return None  # No object found

    def track_object(self):
        """Main loop for tracking the object."""
        while True:
            ret, frame = self.capture.read()
            if not ret:
                print("Failed to capture frame")
                break

            # Get the object's position
            position = self.process_frame(frame)

            if position:
                cv2.circle(frame, position, 10, (0, 255, 0), -1)  # Mark the object
                print(f"Object position: {position}")
                self.move_robot(position)
            else:
                print("Object not detected")

            # Display the frame
            cv2.imshow("Object Tracking", frame)

            # Exit on 'q' key press
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break

        self.capture.release()
        cv2.destroyAllWindows()

    def move_robot(self, position):
        """Move the robot based on the object's position."""
        x, y = position
        cx, cy = self.frame_center

        # Horizontal movement
        if x < cx - self.threshold:
            print("Object to the left, turning left")
            self.robot.body_left(5)  # Adjust step size
        elif x > cx + self.threshold:
            print("Object to the right, turning right")
            self.robot.body_right(5)  # Adjust step size

        # Forward/backward movement
        if y < cy - self.threshold:
            print("Object above, moving forward")
            self.robot.step_forward(1)  # Adjust step size
        elif y > cy + self.threshold:
            print("Object below, moving backward")
            self.robot.step_back(1)  # Adjust step size

if __name__ == "__main__":
    tracker = ObjectTracker()
    tracker.track_object()
S
