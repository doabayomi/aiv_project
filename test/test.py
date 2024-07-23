from wave_rover_serial import Robot

# Initialize the Robot with the appropriate serial port and baud rate
with Robot('/dev/ttyUSB0') as robot:
    # Send a command to set motor speeds
    # robot.speed_input(left_speed=100, right_speed=100)

    # Send a command to get the IMU information and read the response
    data = robot.imu_info()
    # data = robot.oled_set(1, "Hello World")
    print(data)