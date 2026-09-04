import rclpy
from rclpy.node import Node
from std_msgs.msg import Header, Float32MultiArray
from geometry_msgs.msg import Twist
from sensor_msgs.msg import Imu, MagneticField, JointState, BatteryState

import os
import time
import json
import threading
import subprocess
import netifaces

from .base_ctrl import BaseController

def get_all_ips():
    ip_dict = {}
    interfaces = netifaces.interfaces()
    for iface in interfaces:
        if iface == 'lo':
            continue
        addrs = netifaces.ifaddresses(iface)
        inet = addrs.get(netifaces.AF_INET)
        if inet:
            ip_dict[iface] = inet[0]['addr']
    return ip_dict

# ROS node class for bringing up the UGV system and publishing sensor data
class ugv_bringup(Node):
    def __init__(self):
        super().__init__('ugv_bringup')
        # Publishers for IMU data, magnetic field data, odometry, and voltage
        # self.imu_data_raw_publisher_ = self.create_publisher(Imu, "imu/data_raw", 20)
        self.imu_data_raw_publisher_ = self.create_publisher(Imu, "imu/raw", 20)
        self.imu_mag_publisher_ = self.create_publisher(MagneticField, "imu/mag", 20)
        self.odom_publisher_ = self.create_publisher(Float32MultiArray, "odom/odom_raw", 20)
        self.voltage_publisher_ = self.create_publisher(BatteryState, "ugv/voltage", 20)
        self._low_battery_warn_interval = 5.0
        self._last_low_battery_warn = 0.0

        # Subscribe to velocity commands (cmd_vel topic)
        self.cmd_vel_sub_ = self.create_subscription(Twist, "cmd_vel", self.cmd_vel_callback, 20)
        self.zero_vel_count = 0
        self.zero_vel_limit = 5
        # Subscribe to joint states (joint_states topic)
        self.joint_states_sub = self.create_subscription(JointState, 'joint_states', self.joint_states_callback, 20)
        self.last_pt_sent_data = None
        # Subscribe to LED control data (ugv/led_ctrl topic)
        self.led_ctrl_sub = self.create_subscription(Float32MultiArray, 'ugv/led_ctrl', self.led_ctrl_callback, 20)
        
        self.pt_steady_ctrl_sub = self.create_subscription(Float32MultiArray, 'ugv/pt_steady_ctrl', self.pt_steady_ctrl_callback, 20)
        
        self.declare_parameter('serial_port', '/dev/ttyAMA0')
        self.declare_parameter('baud_rate', 115200)
        serial_port_name = self.get_parameter('serial_port').value
        baud_rate = self.get_parameter('baud_rate').value
                        
        # Initialize the base controller with the UART port and baud rate
        self.base_controller = BaseController(serial_port_name, baud_rate)
        # Send an explicit zero-velocity command so the base never keeps a
        # stale motion command across a driver restart
        self.send_zero_velocity()
        request_data = json.dumps({"T":131,"cmd":1}) + "\n"
        self.base_controller.send_command(request_data.encode())        
        # Timer to periodically execute the feedback loop
        self.feedback_thread = threading.Thread(target=self.feedback_loop_thread, daemon=True)
        self.feedback_thread.start()
        self.ip_thread = threading.Thread(target=self.ip_thread_func, daemon=True)
        self.ip_thread.start()

        self.set_ugv_version()

    def set_ugv_version(self):
        model = os.getenv("UGV_MODEL", "ugv_rover")
        ugv_main = 2

        if model == "ugv_rover":
            ugv_main = 2
        elif model == "ugv_beast":
            ugv_main = 3
        elif model == "rasp_rover":
            ugv_main = 1
        else:
            ugv_main = 2

        version_data = json.dumps({"T":900,"main":ugv_main,"module":"0"}) + "\n"
        self.base_controller.send_command(version_data.encode())   
        
    def feedback_loop_thread(self):
        rate = self.create_rate(20)
        while rclpy.ok():
            try:
                data = self.base_controller.feedback_data()
                self.base_controller.base_data = data
                if data and data["T"] == 1001:
                    self.publish_imu_mag()
                    self.publish_odom_raw()
                    self.publish_voltage()
                    self.publish_imu_data_raw()
                
            except Exception as e:
                self.get_logger().error(f"[feedback_loop_thread] error: {e}")
            rate.sleep()

    def ip_thread_func(self):
        last_wlan_ip = None
        last_eth_ip = None

        rate = self.create_rate(20)

        while rclpy.ok():
            ip_map = get_all_ips()
            wlan_ip = ip_map.get("wlan0", "N/A")
            eth_ip = ip_map.get("eth0", "N/A")

            if wlan_ip != last_wlan_ip:
                last_wlan_ip = wlan_ip
                data = json.dumps({'T': '3', 'lineNum': 1, 'Text': f"W:{wlan_ip}"}) + "\n"
                self.base_controller.send_command(data.encode())

            if eth_ip != last_eth_ip:
                last_eth_ip = eth_ip
                data = json.dumps({'T': '3', 'lineNum': 0, 'Text': f"E:{eth_ip}"}) + "\n"
                self.base_controller.send_command(data.encode())

            rate.sleep()

    # Publish IMU data to the ROS topic "imu/data_raw"
    def publish_imu_data_raw(self):
        msg = Imu()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Get the current timestamp
        msg.header.frame_id = "base_link"
        imu_raw_data = self.base_controller.base_data
                
        # Populate the linear acceleration and angular velocity fields degree/s  m/s^2 
        msg.linear_acceleration.x = 9.8 * float(imu_raw_data["ax"]) / 8192
        msg.linear_acceleration.y = 9.8 * float(imu_raw_data["ay"]) / 8192
        msg.linear_acceleration.z = 9.8 * float(imu_raw_data["az"]) / 8192
        
        msg.angular_velocity.x = 3.1415926 * float(imu_raw_data["gx"]) / (16.4 * 180)
        msg.angular_velocity.y = 3.1415926 * float(imu_raw_data["gy"]) / (16.4 * 180)
        msg.angular_velocity.z = 3.1415926 * float(imu_raw_data["gz"]) / (16.4 * 180)
                   
        self.imu_data_raw_publisher_.publish(msg)  # Publish the IMU data
        
    # Publish magnetic field data to the ROS topic "imu/mag"
    def publish_imu_mag(self):
        msg = MagneticField()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()  # Get the current timestamp
        msg.header.frame_id = "base_link"
        imu_raw_data = self.base_controller.base_data

        # Populate the magnetic field data μT
        msg.magnetic_field.x = float(imu_raw_data["mx"]) * 0.15
        msg.magnetic_field.y = float(imu_raw_data["my"]) * 0.15
        msg.magnetic_field.z = float(imu_raw_data["mz"]) * 0.15
              
        self.imu_mag_publisher_.publish(msg)  # Publish the magnetic field data

    # Publish odometry data to the ROS topic "odom/odom_raw" m
    def publish_odom_raw(self):
        odom_raw_data = self.base_controller.base_data
        array = [odom_raw_data["odl"]/100, odom_raw_data["odr"]/100,odom_raw_data["L"], odom_raw_data["R"]]
        msg = Float32MultiArray(data=array)
        self.odom_publisher_.publish(msg)  # Publish the odometry data

    def _maybe_low_battery_warning(self, voltage_value: float):
        if not (0.1 < voltage_value < 9):
            return
        now = time.monotonic()
        if now - self._last_low_battery_warn < self._low_battery_warn_interval:
            return
        self._last_low_battery_warn = now
        threading.Thread(
            target=self._low_battery_warn_worker,
            args=(voltage_value,),
            daemon=True,
        ).start()

    def _low_battery_warn_worker(self, voltage_value: float):
        try:
            subprocess.run(
                ['spd-say', 'low battery'],
                check=False,
                timeout=10,
            )
            data = json.dumps({'T': '3', 'lineNum': 2, 'Text': f"V:{voltage_value}"}) + "\n"
            self.base_controller.send_command(data.encode())
        except Exception as e:
            self.get_logger().error(f"Failed low battery warning: {e}")

    # Publish voltage data to the ROS topic "voltage" v
    def publish_voltage(self):
        voltage_data = self.base_controller.base_data
        msg = BatteryState()
        msg.header = Header()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "base_link"
        msg.voltage = float(voltage_data["v"] / 100)
        msg.percentage = float(voltage_data["v"] / 1260)
        msg.present = True
        self.voltage_publisher_.publish(msg)
        self._maybe_low_battery_warning(msg.voltage)

    # Send a zero-velocity command to the UGV as a JSON string
    def send_zero_velocity(self):
        data = json.dumps({'T': '13', 'X': 0.0, 'Z': 0.0}) + "\n"
        self.base_controller.send_command(data.encode())

    # Callback for processing velocity commands m/s
    def cmd_vel_callback(self, msg):
        linear_velocity = msg.linear.x
        angular_velocity = msg.angular.z

        if linear_velocity == 0.0 and angular_velocity == 0.0:
            self.zero_vel_count += 1
            if self.zero_vel_count > self.zero_vel_limit:
                return  
        else:
            self.zero_vel_count = 0  

        # Apply minimum threshold to angular velocity if linear velocity is zero
        if linear_velocity == 0.0:
            if 0 < angular_velocity < 0.2:
                angular_velocity = 0.2
            elif -0.2 < angular_velocity < 0:
                angular_velocity = -0.2

        # Send the velocity data to the UGV as a JSON string
        data = json.dumps({'T': '13', 'X': linear_velocity, 'Z': angular_velocity}) + "\n"
        self.base_controller.send_command(data.encode())

    def joint_states_callback(self, msg):
        header = {
            'stamp': {
                'sec': msg.header.stamp.sec,
                'nanosec': msg.header.stamp.nanosec,
            },
            'frame_id': msg.header.frame_id,
        }

        # Extract joint positions and convert to degrees
        name = msg.name
        position = msg.position

        x_rad = position[name.index('pt_base_link_to_pt_link1')]
        y_rad = position[name.index('pt_link1_to_pt_link2')]

        x_degree = (180 * x_rad) / 3.1415926
        y_degree = (180 * y_rad) / 3.1415926

        # Send the joint data as a JSON string to the UGV
        joint_data = json.dumps({
            'T': 133, 
            'X': -x_degree, 
            'Y': y_degree, 
            "SPD": 0,
            "ACC": 0,
        }) + "\n"

        if joint_data == self.last_pt_sent_data:
            return  

        self.last_pt_sent_data = joint_data
                
        self.base_controller.send_command(joint_data.encode())

    # Callback for processing LED control commands 0-255
    def led_ctrl_callback(self, msg):
        IO4 = msg.data[0]
        IO5 = msg.data[1]

        IO4 = max(0, min(IO4, 255))
        IO5 = max(0, min(IO5, 255))     
                
        # Send LED control data as a JSON string to the UGV
        led_ctrl_data = json.dumps({
            'T': 132, 
            "IO4": IO4,
            "IO5": IO5,
        }) + "\n"
           
        self.base_controller.send_command(led_ctrl_data.encode())


    def pt_steady_ctrl_callback(self, msg):
        mode = int(msg.data[0])
        y_value = msg.data[1]

        mode = max(0, min(mode, 1)) 
                
        # Send LED control data as a JSON string to the UGV
        pt_steady_ctrl_data = json.dumps({
            'T': 137, 
            "s": mode,
            "y": y_value,
        }) + "\n"
           
        self.base_controller.send_command(pt_steady_ctrl_data.encode())

    # Callback for processing voltage data
    def voltage_callback(self, msg):
        voltage_value = msg.voltage

        # If voltage drops below a threshold, play a low battery warning sound
        if 0.1 < voltage_value < 9:
            try:
                subprocess.run(['spd-say', 'low battery'], check=True)
                data = json.dumps({'T': '3', 'lineNum': 2, 'Text': f"V:{voltage_value}"}) + "\n"
                self.base_controller.send_command(data.encode())
            except Exception as e:
                self.get_logger().error(f"Failed to say low battery warning: {e}")
            time.sleep(5)
                                    
# Main function to initialize the ROS node and start spinning
def main(args=None):
    rclpy.init(args=args)  # Initialize ROS
    node = ugv_bringup()  # Create the UGV bringup node
    try:
        rclpy.spin(node)  # Keep the node running
    finally:
        node.send_zero_velocity()  # Stop the base before exiting
        node.base_controller.close()  # Close the serial port
        node.destroy_node()  # Shutdown the node
        rclpy.shutdown()  # Shutdown ROS

if __name__ == '__main__':
    main()
