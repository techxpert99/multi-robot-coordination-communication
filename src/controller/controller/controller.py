from math import asin,acos,atan2,sqrt,pi
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data

def EuclideanDistance(source, target):
    dx = source[0]-target[0]
    dy = source[1]-target[1]
    return sqrt(dx*dx+dy*dy)

def PositiveAngle(angle):
	return (angle+2*pi)%(2*pi)

def AngleDifference(source_angle, target_angle):
    diff = target_angle - source_angle
    if diff>=0:
        adiff = diff
        sgn = -1
    else:
        adiff = -diff
        sgn = 1
    return diff if adiff <= pi else sgn*(2*pi-adiff)

# Transforms a quaternion to roll,pitch,yaw
def TransformQuaternion(x, y, z, w):
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

# Unmarshal location from a message
def UnmarshalPosition(msg: Odometry):
    x,y = msg.pose.pose.orientation.x,msg.pose.pose.orientation.y
    z,w = msg.pose.pose.orientation.z,msg.pose.pose.orientation.w
    roll,pitch,yaw = TransformQuaternion(x,y,z,w)
    return (x,y,yaw)


class Controller:
    
    def __init__(self, parameter_store, robot_namespace, linear_speed, angular_speed, linear_precision=0.1, angular_precision=0.02):
        
        self.node = Node(robot_namespace+'/controller')
        self.parameter_store = parameter_store
        
        self.velocity_publisher = self.node.create_publisher(Twist,robot_namespace+'/velocity',qos_profile_system_default)
        self.odometry_subscription = self.node.create_subscription(Odometry,robot_namespace+'/odometry',self.UpdatePosition,qos_profile_sensor_data)
        
        self.position = None
        self.plan = None
        self.plan_index = None
        self.target = None
        self.velocity = None
        self.initial_point = None
        self.initial_index = None

        self.linear = linear_speed
        self.angular = angular_speed
        self.linear_precision = linear_precision
        self.angular_precision = angular_precision

        self.parameter_store.set('controller','position',None)
        self.parameter_store.set('controller','plan',None)
        self.parameter_store.set('controller','plan_changed',False)

    def UpdatePosition(self, msg: Odometry):
        self.position = UnmarshalPosition(msg)
        self.parameter_store.set('controller','position',self.position)
    
    def ComputeVelocity(self, maximum_linear_speed=None, maximum_angular_speed=None, distance_precision = None):

        target_reached = True
        velocity = Twist()

        if not self.position or not self.target:
            self.velocity = velocity
            return
        
        source = self.position
        target = self.target

        distance = EuclideanDistance(source, target)
        distance_precision = distance_precision or self.linear_precision

        if distance >= distance_precision:
            displacement = [target[0]-source[0],target[1]-source[1]]
            angle = acos(displacement[0]/distance)
            angle = angle if displacement[1] >= 0 else -angle
            target_angle = AngleDifference(source[2],angle)
            if distance >= distance_precision*2 and abs(target_angle) >= self.angular_precision*5:
                velocity.linear.x = 0.
                velocity.angular.z = self.angular * target_angle
                if maximum_angular_speed is not None:
                    velocity.angular.z *= min(abs(velocity.angular.z), maximum_angular_speed)/ abs(velocity.angular.z)
            else:
                velocity.linear.x = self.linear
                velocity.angular.z = 0.
                if maximum_linear_speed is not None:
                    velocity.linear.x = min(velocity.linear.x, maximum_linear_speed)
            target_reached = False

        if target_reached and target[2] is not None:
            target_angle = AngleDifference(source[2],target[2])
            if abs(target_angle) >= self.angular_precision:
                velocity.linear.x = 0.
                velocity.angular.z = self.angular * target_angle
                if maximum_angular_speed is not None:
                    velocity.angular.z *= min(abs(velocity.angular.z), maximum_angular_speed) / abs(velocity.angular.z)
                target_reached = False

        self.velocity = velocity
        self.target_reached = target_reached
    
    def UpdatePlan(self):
        
        if self.parameter_store.get('plan_changed'):
            self.plan = self.parameter_store.get('controller','plan')
            self.parameter_store.set('controller','plan_changed',False)
            self.plan_index = None
            self.initial_point = None
            self.initial_index = None
        
        if self.plan is not None and self.plan_index is None and self.position:
            
            px,py,_ = self.position
            cache = None

            for i in range(1,len(self.plan)):

                ax,ay = self.plan[i-1]
                bx,by = self.plan[i]
                
                if bx == ax:
                    entry = (abs(ax-px),(ax,by),i)
                
                else:
                    m = (by-ay)/(bx-ax)
                    x = (m*(py-ay)+m*m*ax+px)/(1+m*m)
                    y = m*(x-ax)+ay
                    d = EuclideanDistance((px,py),(x,y))
                    entry = (d,(x,y),i)

                if cache is None or entry[0] < cache[0]:
                    cache = entry

            self.initial_point = cache[1]
            self.initial_index = cache[2]
            self.plan_index = -1

    def DecideTarget(self):

        if self.plan_index == -1:
            self.target = self.initial_point
            self.plan_index = self.initial_index
        elif self.target_reached:
            if self.plan_index < len(self.plan):
                self.target = self.plan[self.plan_index]
                self.plan_index += 1
            else:
                self.plan = None
                self.plan_index = None
                self.target = None
                self.initial_index = None
                self.initial_point = None
                self.parameter_store.set('controller','plan',None)

    def ExecutePlan(self):
        self.UpdatePlan()
        self.DecideTarget()
        self.ComputeVelocity()
        if self.velocity is not None:
            self.velocity_publisher.publish(self.velocity)

    def RunController(self):
        self.UpdatePosition()
        self.ExecutePlan()

    def DestroyController(self):
        self.node.destroy_node()