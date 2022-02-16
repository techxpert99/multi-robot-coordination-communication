from math import sqrt,floor,ceil,atan2,asin,pi
from geometry_msgs.msg import Twist
from time import sleep
import numpy as np


def euler_from_quaternion(x, y, z, w):
    t0 = +2.0 * (w * x + y * z)
    t1 = +1.0 - 2.0 * (x * x + y * y)
    roll_x = atan2(t0, t1)

    t2 = +2.0 * (w * y - z * x)
    t2 = +1.0 if t2 > +1.0 else t2
    t2 = -1.0 if t2 < -1.0 else t2
    pitch_y = asin(t2)

    t3 = +2.0 * (w * z + x * y)
    t4 = +1.0 - 2.0 * (y * y + z * z)
    yaw_z = atan2(t3, t4)
    return roll_x, pitch_y, yaw_z # in radians

def velocity_to_message(linear, angular):
    msg = Twist()
    msg.linear.x = linear
    msg.angular.z = angular
    return msg

def velocity_from_message(msg: Twist):
        return msg.linear.x, msg.angular.z

def wait(time_in_seconds):
    if time_in_seconds > 0:
        sleep(time_in_seconds)

def merge_bounding_boxes(bb1,bb2):
    return min(bb1[0],bb2[0]),min(bb1[1],bb2[1]),max(bb1[2],bb2[2]),max(bb1[3],bb2[3])

def constrain(value,min_value,max_value):
    if value < min_value: return min_value
    elif value > max_value: return max_value
    return value

def sgn(x):
    if x > 0: return 1
    elif x < 0: return -1
    return 0

def normalize_angle(angle):
    two_pi = 2*pi
    angle %= two_pi
    if angle < -pi:
        return angle+two_pi
    elif angle > pi:
        return angle-two_pi
    return angle

def cartesian_dist(point1,point2):
    return sqrt((point1[0]-point2[0])**2+(point1[1]-point2[1])**2)

def save_critical_map(path,cmap:np.ndarray):
    np.save(path,cmap)

def load_critical_map(path):
    return np.load(path)