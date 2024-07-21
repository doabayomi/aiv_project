import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, Vector3
from sensor_msgs.msg import Imu
from wave_rover_serial import Robot
import math
import json

class VehicleNode(Node):
    def __init__(self):
        super().__init__('vehicle_node')

        self.declare_parameter('timer_period', 1)
        # self.declare_parameter('frame_id', "laser_frame")

        # Subscribe to the /cmd_vel topic to receive velocity commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Publish IMU data to the /imu_data topic
        self.imu_data_publisher = self.create_publisher(
            Imu,
            '/imu',
            10)
        
        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        # self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        self.timer = self.create_timer(timer_period, self.timer_callback)

        # Initialize connection to the vehicle
        self.rover = Robot('/dev/ttyUSB0')
        self.rover.connect()

    def timer_callback(self):
        self.publish_imu_data()

    def cmd_vel_callback(self, msg: Twist):
        """
        Process velocity commands received from /cmd_vel topic
        """
        # Convert Twist message to JSON format compatible with wave_rover_serial
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        wheelbase = 0.136 # in meters
        left_speed = linear_velocity - angular_velocity * wheelbase/2
        right_speed = linear_velocity + angular_velocity * wheelbase / 2

        # Keeping the speed input in acceptable range
        left_speed = max(min(left_speed, 255), -255)
        right_speed = max(min(right_speed, 255), -255)

        self.rover.speed_input(left_speed, right_speed)
        
    
    def publish_imu_data(self):
        """
        Publishes the imu data to a the /imu_data topic
        """

        # Initialize the Robot with the appropriate serial port and baud rate
        # with Robot('/dev/ttyUSB0') as robot:
        #     # Send a command to set motor speeds
        #     # robot.speed_input(left_speed=100, right_speed=100)

        #     # Send a command to get the IMU information and read the response
        #     imu_data = robot.imu_info()
            # print(data)
        # Get IMU data from the vehicle
        imu_data = self.rover.imu_info()

        if imu_data == "Read timeout":
            imu_data = json.loads(imu_data)
    
        
        # Create an Imu message
        imu_msg = Imu()
        
        # # Populate header (timestamp)
        # # imu_msg.header.frame_id = self.frame_id
        # imu_msg.header.stamp = self.get_clock().now().to_msg()py
        
        # # Populate orientation (quaternion)
        # quaternion = self.quaternion_from_rpy(math.radians(imu_data['roll']),
        #                                       math.radians(imu_data['pitch']),
        #                                       math.radians(imu_data['yaw']))
        # imu_msg.orientation.x = quaternion[0]
        # imu_msg.orientation.y = quaternion[1]
        # imu_msg.orientation.z = quaternion[2]
        # imu_msg.orientation.w = quaternion[3]

        
        # # Populate orientation covariance
        # imu_msg.orientation_covariance = [0.0] * 9  # Assuming no covariance for orientation
        
        # # Populate angular velocity
        # # imu_msg.angular_velocity = Vector3()
        # imu_msg.angular_velocity.x = float(imu_data['gyro_X'])
        # imu_msg.angular_velocity.y = float(imu_data['gyro_Y'])
        # imu_msg.angular_velocity.z = float(imu_data['gyro_Z'])
        
        # # Populate angular velocity covariance
        # imu_msg.angular_velocity_covariance = [0.0] * 9  # Assuming no covariance for angular velocity
        
        # # Populate linear acceleration
        # # imu_msg.linear_acceleration = Vector3()
        # imu_msg.linear_acceleration.x = float(imu_data['acce_X'])
        # imu_msg.linear_acceleration.y = float(imu_data['acce_Y'])
        # imu_msg.linear_acceleration.z = float(imu_data['acce_Z'])
        
        # # Populate linear acceleration covariance
        # imu_msg.linear_acceleration_covariance = [0.0] * 9  # Assuming no covariance for linear acceleration
        
        # Publish IMU message
        self.imu_data_publisher.publish(imu_msg)
    
    def quaternion_from_rpy(self, roll, pitch, yaw):
        """
        Convert roll, pitch, yaw to quaternion.
        """
        cy = math.cos(yaw * 0.5)
        sy = math.sin(yaw * 0.5)
        cp = math.cos(pitch * 0.5)
        sp = math.sin(pitch * 0.5)
        cr = math.cos(roll * 0.5)
        sr = math.sin(roll * 0.5)

        qw = cy * cp * cr + sy * sp * sr
        qx = cy * cp * sr - sy * sp * cr
        qy = sy * cp * sr + cy * sp * cr
        qz = sy * cp * cr - cy * sp * sr

        return [qx, qy, qz, qw]
    
    def disconnect(self):
        # Disconnect from the vehicle when shutting down the node
        self.rover.disconnect()

def main(args=None):
    rclpy.init(args=args)
    vehicle_node = VehicleNode()
    
    try:
        rclpy.spin(vehicle_node)
    except KeyboardInterrupt:
        pass
    
    vehicle_node.disconnect()
    vehicle_node.destroy_node()
    # rclpy.shutdown()

if __name__ == '__main__':
    main()