import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

def cb1(msg: LaserScan):
    print(msg)

def main(args=None):
    rclpy.init()
    node = Node('controller')
    node.create_subscription(LaserScan, '/demo/laser/out', cb1, qos_profile_sensor_data)
    rclpy.spin()
    rclpy.shutdown()