# Project-Legion-Robot
 This is a 12 DOF Robot Based on RPI/Arduino/ESP32
    
This Repo Have Various Version of the Robot (Basicly All you need)

1. Arduino Without Servo Driver (This is the Simplest Version of Legion with Ardduino Nano And Nano Expansion Used Serial Monitor To Control)
2. Arduino With Servo Driver (This Version Use 16 Channel PCA Servo Driver WITH Arduino Solving pins issue, USE SDA and SCL)
3. Arduino With ESPCam  ("This Version Use Espcam and arduino to give live view and control interface, Use Rx Tx")
4. ESP32 (Since FlexiTimer2.h is not working on ESP Boards, i need to Find another way to Work on this Version any, 'IF ANY IDEA PLEASE CONTRIBUTE')
5. raspberri pi (This is The python Version, I am Using RPI 4 and Servo Driver for AI and ML Integration)

What to DO ?
1. 3d Print the Structure and Pick your Code Version
2. attach the servos to the structure (Use Servo Pin image for pin connection in media folder) (*without Servo horn)
3. Use Servo_Config Code to calibrate all the servo to 90
4. Attach all the servo horn maintaining the initial position (Use Initial position image in media folder) (*IMPORTANT)
5. Complete the Circuits and Upload/Run main code and enjoy

![Screenshot 2024-12-06 111144](https://github.com/user-attachments/assets/2d18a6cd-879f-407e-b778-d44b60fc827b)

Note: 
Main Logic I used Was by Sunfounder Robot
panerqiang@sunfounder.com
