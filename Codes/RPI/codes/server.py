from flask import Flask, render_template, request, jsonify
from kinematics import RobotKinematics  # Import your robot library
import threading
import time

app = Flask(__name__)
robot = RobotKinematics()  # Initialize your robot object

# Variable to control the looping command
current_command = None
looping_thread = None
command_running = False  # Flag to control the loop


# Function to execute looping commands
def command_loop():
    global command_running
    while command_running:
        if current_command == "forward":
            robot.step_forward(1)
        elif current_command == "backward":
            robot.step_back(1)
        elif current_command == "left":
            robot.turn_left(1)
        elif current_command == "right":
            robot.turn_right(1)
        time.sleep(0.1)  # Adjust the delay as needed


@app.route('/')
def home():
    return render_template('index.html')  # Serve the control interface


@app.route('/command', methods=['POST'])
def handle_command():
    global current_command, command_running, looping_thread
    data = request.json
    command = data.get('action', '')
    if command:
        print(f"Received command: {command}")
        
        # Handle start of continuous commands
        if command in ["forward", "backward", "left", "right"]:
            if current_command != command:
                # Update the current command and start looping
                current_command = command
                if not command_running:
                    command_running = True
                    looping_thread = threading.Thread(target=command_loop)
                    looping_thread.start()
        
        # Handle stop command
        elif command == "stop":
            command_running = False  # Stop the loop
            current_command = None
            robot.stand()  # Example for stopping
            
        # Handle one-time commands
        elif command == "handshake":
            robot.hand_shake(3)
        elif command == "handwave":
            robot.hand_wave(3)
        elif command == "sit":
            robot.sit()
        elif command == "dance":
            robot.body_dance(5)
        
        return jsonify({"status": "success"}), 200
    return jsonify({"status": "error"}), 400


if __name__ == '__main__':
    # Setup robot in a separate thread to avoid blocking
    threading.Thread(target=lambda: (robot.setup(), robot.stand()), daemon=True).start()
    app.run(host='0.0.0.0', port=5000)
