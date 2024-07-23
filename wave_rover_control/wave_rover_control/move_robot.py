import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Quaternion, Vector3
from sensor_msgs.msg import Imu, MagneticField, Temperature
from std_srvs.srv import Empty
from wave_rover_interfaces.srv import OledDisplay
from wave_rover_serial import Robot
import math
import json

class VehicleNode(Node):
    def __init__(self):
        super().__init__('vehicle_node')

        self.declare_parameter('timer_period', 1)
        self.declare_parameter('frame_id', "base_link")
        self.declare_parameter('port', "/dev/ttyUSB0")

        # Subscribe to the /cmd_vel topic to receive velocity commands
        self.cmd_vel_subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # # Publish IMU data to the /imu_data topic
        self.imu_data_publisher = self.create_publisher(
            Imu,
            '/imu/data_raw',
            10)
        
        self.mag_publisher = self.create_publisher(
            MagneticField,
            'imu/mag',
            10
        )

        self.temp_publisher = self.create_publisher(
            Temperature,
            'imu/temprature',
            10
        )

        self.emergency_stop_service = self.create_service(Empty, 'emergency_stop', self.emergency_stop)

        self.oled_display_service = self.create_service(
            OledDisplay,
            'oled_display_text',
            self.oled_display
        )
        
        timer_period = self.get_parameter('timer_period').get_parameter_value().double_value
        port = self.get_parameter('port').get_parameter_value().string_value
        self.frame_id = self.get_parameter('frame_id').get_parameter_value().string_value

        # Initialize connection to the vehicle
        self.rover = Robot(port)
        self.rover.connect()

        self.timer = self.create_timer(timer_period, self.timer_callback)

    def oled_display(self, request, response):
        self.rover.oled_set(request.line_num, request.text)

        response.resp = True
        return response
    
    def emergency_stop(self, request, response):
        self.rover.emergency_stop()

        return response

    def timer_callback(self):
        # with Robot('/dev/ttyUSB0') as robot:
        #     # Send a command to set motor speeds
        #     # robot.speed_input(left_speed=100, right_speed=100)

        #     # Send a command to get the IMU information and read the response
        #     imu_data = robot.imu_info()
        imu_data = self.rover.imu_info()
        # print(imu_data)
        # if imu_data != "Read timeout.":
        try:
            imu_data = json.loads(imu_data)

            self.publish_imu_data(imu_data)
        except:
            self.get_logger().info(imu_data)

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

        left_pwm = (left_speed / 5.0) * 255
        right_pwm = (right_speed / 5.0) * 255 

        # Keeping the speed input in acceptable range
        left_pwm = max(min(left_pwm, 255), -255)
        right_pwm = max(min(right_pwm, 255), -255)

        self.get_logger().info(f"{left_pwm} {right_pwm}")
        # self.get(f"{left_pwm} {right_pwm}")

        self.rover.speed_input(left_pwm, right_pwm)
        # self.rover.speed_input(left_speed=100, right_speed=100)

        # self.rover.
    
    def publish_imu_data(self, imu_data):
        """
        Publishes the imu data to a the /imu_data topic
        """
        # Create an Imu message
        imu_msg = Imu()
        
        # Populate header (timestamp)
        imu_msg.header.frame_id = self.frame_id
        imu_msg.header.stamp = self.get_clock().now().to_msg()
        
        # Populate orientation (quaternion)
        quaternion = self.quaternion_from_rpy(math.radians(imu_data['roll']),
                                              math.radians(imu_data['pitch']),
                                              math.radians(imu_data['yaw']))
        imu_msg.orientation.x = quaternion[0]
        imu_msg.orientation.y = quaternion[1]
        imu_msg.orientation.z = quaternion[2]
        imu_msg.orientation.w = quaternion[3]

        
        # Populate orientation covariance
        imu_msg.orientation_covariance = [0.0] * 9  # Assuming no covariance for orientation
        
        # Populate angular velocity
        # imu_msg.angular_velocity = Vector3()
        imu_msg.angular_velocity.x = float(imu_data['gyro_X'])
        imu_msg.angular_velocity.y = float(imu_data['gyro_Y'])
        imu_msg.angular_velocity.z = float(imu_data['gyro_Z'])
        
        # Populate angular velocity covariance
        imu_msg.angular_velocity_covariance = [0.0] * 9  # Assuming no covariance for angular velocity
        
        # Populate linear acceleration
        # imu_msg.linear_acceleration = Vector3()
        imu_msg.linear_acceleration.x = float(imu_data['acce_X'])
        imu_msg.linear_acceleration.y = float(imu_data['acce_Y'])
        imu_msg.linear_acceleration.z = float(imu_data['acce_Z'])
        
        # Populate linear acceleration covariance
        imu_msg.linear_acceleration_covariance = [0.0] * 9  # Assuming no covariance for linear acceleration
        

        # Populate Magnectometer reading
        Mag = MagneticField()

        Mag.header = imu_msg.header
        Mag.magnetic_field.x = float(imu_data["magn_X"])
        Mag.magnetic_field.y = float(imu_data["magn_Y"])
        Mag.magnetic_field.z = float(imu_data["magn_Z"])
        Mag.magnetic_field_covariance = [0.0] * 9  # Assuming no covariance for linear acceleration

        # Populate temperature 
        Temp_msg = Temperature()
        Temp_msg.header = imu_msg.header
        Temp_msg.temperature = float(imu_data['temp'])

        # Publish IMU message
        self.mag_publisher.publish(Mag)
        self.imu_data_publisher.publish(imu_msg)
        self.temp_publisher.publish(Temp_msg)
    
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