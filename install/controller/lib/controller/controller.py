from math import acos, asin, atan, atan2, ceil, cos, floor, isinf, sin, sqrt
import math
from numpy.core.defchararray import startswith
from numpy.lib.arraysetops import isin
from numpy.lib.function_base import select
import rclpy
from rclpy.context import Context
from rclpy.node import Node
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from rclpy.timer import Rate
from sensor_msgs.msg import LaserScan
import cv2
import numpy as np
from math import pi
from nav_msgs.msg import Odometry
from time import sleep
from threading import Lock
from queue import PriorityQueue, Queue
from threading import Thread

from concurrent.futures import ThreadPoolExecutor
import os

from examples_rclpy_executors.listener import Listener
from examples_rclpy_executors.talker import Talker
import rclpy
from rclpy.executors import Executor, MultiThreadedExecutor
from rclpy.node import Node
from std_msgs.msg import String

class SafeParameterStore:
    
    def __init__(self):
        self.__store__ = dict()
        self.__lock__ = Lock()
    
    def register(self,name):
        self.__lock__.acquire()
        self.__store__[name] = dict(),Lock()
        self.__lock__.release()
        
    def set(self,store,key,value):
        self.__store__[store][1].acquire()
        self.__store__[store][0][key] = value
        self.__store__[store][1].release()
    
    def get(self,store,key):
        self.__store__[store][1].acquire()
        value = self.__store__[store][0][key]
        self.__store__[store][1].release()
        return value
    
    def has(self,store,key):
        self.__store__[store][1].acquire()
        out = key in self.__store__[store][0]
        self.__store__[store][1].release()
        return out
    
    def lock(self,store):
        self.__store__[store][1].acquire()
    
    def release(self,store):
        self.__store__[store][1].release()

    def unsafe_get(self,store,key):
        return self.__store__[store][0][key]
    
    def unsafe_set(self,store,key,value):
        self.__store__[store][0][key] = value


STORE = SafeParameterStore()

STORE.register('general')
STORE.register('critical')
STORE.register('estimator')
STORE.register('controller')
STORE.register('mapper')
STORE.register('sensor')
STORE.register('planner')

STORE.set('general','topics',{'laser.in': ('/demo/laser/out',LaserScan,qos_profile_sensor_data), 'odom.in': ('/demo/odom',Odometry,10), 'vel.out':('/demo/cmd_vel',Twist,10)})


class NodeWrapper:

    def __init__(self, name):
        self.__node__ = Node(name)
        EXECUTOR.add_node(self.__node__)

    def create_subscription(self,topic,callback):
        topic_name,topic_type,topic_profile = STORE.get('general','topics')[topic]
        return self.__node__.create_subscription(topic_type,topic_name,callback,topic_profile)
    
    def create_publisher(self,topic):
        topic_name,topic_type,topic_profile = STORE.get('general','topics')[topic]
        return self.__node__.create_publisher(topic_type,topic_name,topic_profile)
    
    def destroy(self):
        self.__node__.destroy_node()


class Auxiliary:

    def get_image_from_local_obstacle_map(obs_map,size = None, thickness = 5):
        obs_map = obs_map.get_map()
        if size == None:
            size = (len(obs_map),len(obs_map[0]))
        
        im = np.ones((size[0],size[1],3),np.uint8)*255
        scale_i,scale_j = size[0]/len(obs_map), size[1]/len(obs_map[1])
        for i in range(size[0]):
            for j in range(size[1]):
                if obs_map[floor(i/scale_i)][floor(j/scale_j)]:
                    im[i][j] = [0,0,0]

        return im

    def construct_image_from_obstacle_map(obs_map: np.ndarray, size = None, thickness = 5):
        if size == None:
            size = obs_map.shape
        im = np.ones((size[0],size[1],3),np.uint8)*255
        scale_i,scale_j = size[0]/obs_map.shape[0], size[1]/obs_map.shape[1]

        def get_color(i,j):
            c,cmax = 0,0
            xmin,xmax=max(0,floor(i-thickness/2)),min(obs_map.shape[1],ceil(i+thickness/2))
            ymin,ymax=max(0,floor(j-thickness/2)),min(obs_map.shape[0],ceil(j+thickness/2))
            epsilon = 10**10
            for k in range(xmin,xmax):
                for l in range(ymin,ymax):
                    d = (i-k)**2+(j-l)**2 + epsilon
                    cmax += d
                    c += obs_map[k,l]*d
            return [round(255*(1-(c+epsilon)/(cmax+epsilon))) for i in range(3)]

        for j in range(size[0]):
            for i in range(size[1]):
                im[j,i] = get_color(i/scale_i,j/scale_j)
        return im


    def euler_from_quaternion(x, y, z, w):
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
    
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
    
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
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


class BitField:
    
    def __init__(self, bit_field_size_in_bits, word_size_in_bits = 32):
        self.__bit_field__ = [0]*ceil(bit_field_size_in_bits/word_size_in_bits)
        self.__word_size__ = word_size_in_bits
        self.__length__ = bit_field_size_in_bits
    
    def bit_modify(self, index, value):
        i = index//self.__word_size__
        j = index%self.__word_size__
        self.__bit_field__[i] ^= (((self.__bit_field__[i]>>j)&1)^value)<<j
    
    def bit_set(self, index):
        self.__bit_field__[index//self.__word_size__] |= 1<<(index%self.__word_size__)

    def bit_unset(self, index):
        i = index//self.__word_size__
        j = index%self.__word_size__
        self.__bit_field__[i] ^= ((self.__bit_field__[i]>>j)&1)<<j
    
    def bit_get(self, index):
        return (self.__bit_field__[index//self.__word_size__]>>(index%self.__word_size__))&1
    
    def for_each(self, callback):
        for index in range(self.__length__):
            callback(index,self.bit_get(index))


class Patch:
    
    def __init__(self, msg: LaserScan, position):
        self.__obstacle_map__ = BitField(len(msg.ranges))
        self.__obstacles__ = []
        self.__origin__ = position
        for i,d in enumerate(msg.ranges):
            if not isinf(d):
                self.__obstacle_map__.bit_set(i)
                self.__obstacles__.append(d)

    def for_each_obstacle(self,callback):
        i = 0
        def callback2(t,b):
            nonlocal i
            if b:
                callback(self.__obstacles__[i],t)
                i += 1
        self.__obstacle_map__.for_each(callback2)


    def rasterize(self, sensor_radius, resolution, obstacle_lookup_radius):
        
        cx,cy,ct = self.__origin__
        xmin,ymax = cx-sensor_radius,cy+sensor_radius
        
        row = round(2*sensor_radius/resolution)
        col = row
        
        obstacle_map = [None]*181
        j=0
        for i in range(181):
            if self.__obstacle_map__.bit_get(i):
                obstacle_map[i] = self.__obstacles__[j]
                j += 1

        rasterized_patch = np.full(shape=(row,col),dtype=np.uint8,fill_value=2)

        for i in range(row):
            for j in range(col):
                
                x,y = xmin-cx+resolution/2+j*resolution,ymax-cy-resolution/2-i*resolution
                r,t = sqrt(x*x+y*y),Auxiliary.normalize_angle(atan2(y,x)-ct+pi/2)

                if t < 0: pass
                elif r == 0:
                    rasterized_patch[i,j] = 0
                else:
                    if obstacle_lookup_radius > r:
                        tmin,tmax = 0,pi
                    else:
                        dt = atan(obstacle_lookup_radius/r)
                        if t < pi/2:
                            tmin,tmax = max(t-dt,0),t+dt
                        else:
                            tmin,tmax = t-dt,min(t+dt,pi)
                    begin,end = ceil(tmin*180/pi),floor(tmax*180/pi)
                    for t1 in range(begin,end+1):
                        r1 = obstacle_map[t1]
                        if r1 is not None:
                            d = r*cos(t-t1*pi/180)
                            if r*r+r1*r1-obstacle_lookup_radius*obstacle_lookup_radius <= 2*r1*d:
                                rasterized_patch[i,j] = 1
                                break
                            if d < r1:
                                rasterized_patch[i,j] = 0
                        else:
                            rasterized_patch[i,j] = 0
        
        return (xmin,cy-sensor_radius,cx+sensor_radius,ymax),rasterized_patch


class Map:
    
    def __init__(self,sensor_radius,resolution,obstacle_lookup_radius):
        self.__map__ = None
        self.__sensor_radius__ = sensor_radius
        self.__resolution__ = resolution
        self.__obstacle_lookup_radius__ = obstacle_lookup_radius
    
    def __transform__(self,i,j,xmin,ymax):
        return xmin+(j+0.5)*self.__resolution__,ymax-(i+0.5)*self.__resolution__
    
    def __inverse_transform__(self,x,y,xmin,ymax):
        return floor((ymax-y)/self.__resolution__),floor((x-xmin)/self.__resolution__)
    
    def merge_patch(self, patch: Patch):

        if self.__map__ is None:
            self.__bounds__,self.__map__ = patch.rasterize(self.__sensor_radius__,self.__resolution__,self.__obstacle_lookup_radius__)
            return

        patch_bounds,rasterized_patch = patch.rasterize(self.__sensor_radius__,self.__resolution__,self.__obstacle_lookup_radius__)
        merged_bounds = Auxiliary.merge_bounding_boxes(self.__bounds__,patch_bounds)
        merged_num_rows,merged_num_cols = floor((merged_bounds[3]-merged_bounds[1])/self.__resolution__),floor((merged_bounds[2]-merged_bounds[0])/self.__resolution__)
        merged_map = np.full(shape=(merged_num_rows,merged_num_cols),dtype=np.uint8,fill_value=2)

        for i in range(self.__map__.shape[0]):
            for j in range(self.__map__.shape[1]):
                if self.__map__[i,j] != 2:
                    t1 = self.__transform__(i,j,self.__bounds__[0],self.__bounds__[3])
                    t2 = self.__inverse_transform__(t1[0],t1[1],merged_bounds[0],merged_bounds[3])
                    k = Auxiliary.constrain(t2[0],0,merged_num_rows-1)
                    l = Auxiliary.constrain(t2[1],0,merged_num_cols-1)
                    merged_map[k,l] = self.__map__[i,j]

        for i in range(rasterized_patch.shape[0]):
            for j in range(rasterized_patch.shape[1]):
                if rasterized_patch[i,j] != 2:
                    t1 = self.__transform__(i,j,patch_bounds[0],patch_bounds[3])
                    t2 = self.__inverse_transform__(t1[0],t1[1],merged_bounds[0],merged_bounds[3])
                    k = Auxiliary.constrain(t2[0],0,merged_num_rows-1)
                    l = Auxiliary.constrain(t2[1],0,merged_num_cols-1)
                    merged_map[k,l] = rasterized_patch[i,j]

        self.__bounds__= merged_bounds
        self.__map__ = merged_map
    
    def get_map(self): return self.__map__


class CriticalMap:
    
    def __init__(self,map: Map,critical_radius):
        self.__map__ = map
        self.__critical_radius__ = critical_radius
        self.__critical_map__ = None
        self.update_critical_map()

    def update_critical_map(self):
        matrix = self.__map__.get_map()
        if matrix is None: return
        rows,cols = matrix.shape
        self.__bounds__ = self.__map__.__bounds__[0]-self.__critical_radius__,self.__map__.__bounds__[1]-self.__critical_radius__,self.__map__.__bounds__[2]+self.__critical_radius__,self.__map__.__bounds__[3]+self.__critical_radius__
        self.__rows__,self.__columns__ = ceil((self.__bounds__[3]-self.__bounds__[1])/self.__map__.__resolution__),ceil((self.__bounds__[2]-self.__bounds__[0])/self.__map__.__resolution__)
        critical_map = np.full((self.__rows__,self.__columns__),2,np.uint8)
        delta = ceil(self.__critical_radius__/self.__map__.__resolution__)
        extention = ceil(2*self.__critical_radius__/self.__map__.__resolution__)
        for i in range(rows):
            for j in range(cols):
                if matrix[i,j] == 1:
                    for k in range(i,1+min(i+extention,self.__rows__-1)):
                        for l in range(j,1+min(j+extention,self.__columns__-1)):
                            critical_map[k,l] = 1
                elif matrix[i,j] == 0:
                    critical_map[min(i+delta,self.__rows__-1),min(self.__columns__-1,j+delta)] = 0
        self.__critical_map__ = critical_map
    
    def get_critical_map(self):
        return self.__critical_map__

    def tf(self,i,j):
        xmin = self.__bounds__[0]
        ymax = self.__bounds__[3]
        return xmin+(j+0.5)*self.__map__.__resolution__,ymax-(i+0.5)*self.__map__.__resolution__
    
    def itf(self,x,y):
        xmin = self.__bounds__[0]
        ymax = self.__bounds__[3]
        return floor((ymax-y)/self.__map__.__resolution__), floor((x-xmin)/self.__map__.__resolution__)


class ThreadWrapper:

    def __init__(self,callback,interval):
        self.__lock__ = Lock()
        self.__run__ = True
        self.__interval__ = interval
        self.__callback__ = callback
        self.__thread__ = Thread(target=self.__target__)
    
    def __target__(self):
        while self.__runnable__():
            self.__callback__()
            Auxiliary.wait(self.__interval__)
    
    def __runnable__(self):
        self.__lock__.acquire()
        runnable = self.__run__
        self.__lock__.release()
        return runnable
    
    def start(self):
        self.__thread__.start()

    def stop(self):
        self.__lock__.acquire()
        self.__run__ = False
        self.__lock__.release()
        self.__thread__.join()


class Curve:

    def __init__(self,resolution=0.000001):
        self._resolution = resolution
        
    def radius_of_curvature(self,x):
        d1 = abs(self.derivative(x,1))
        d2 = abs(self.derivative(x,2))
        return (1+d1*d1)**1.5/d2

    def derivative(self,x,order):
        if order < 0: return
        cache = np.zeros((order+1,order+1),np.float64)
        for j in range(order+1):
            cache[0,j] = self.at(x+j*self._resolution)
        for i in range(1,order+1):
            for j in range(order+1-i):
                cache[i,j] = (cache[i-1,j+1]-cache[i-1,j])/self._resolution
        return cache[order,0]


class Polynomial(Curve):
    
    def __init__(self,data_points,high_precision=False):
        Curve.__init__(self)
        degree_plus_1 = len(data_points)
        X = np.zeros((degree_plus_1,degree_plus_1),np.float64)
        Y = np.zeros((degree_plus_1,1),np.float64)
        for i,point in enumerate(data_points):
            Y[i,0] = point[1]
            element = 1
            for j in range(degree_plus_1):
                X[i,j] = element
                element *= point[0]
        self._c = np.reshape(np.matmul(np.linalg.inv(X),Y),(degree_plus_1))
        self._hp = False
        if high_precision:
            self._s = np.ones((degree_plus_1),np.byte)
            for i in range(1,degree_plus_1):
                if self._c[i] >= 0:
                    self._c[i] **= 1/i
                else:
                    self._c[i] = (-self._c[i])**1/i
                    self._s[i] = -1
            self._hp = True
    
    def at(self,point):
        if not self._hp:
            y,p=0,1
            for c in self._c:
                y += p*c
                p *= point
        else:
            y=self._c[0]
            for i in range(1,len(self._c)):
                y += self._s[i]*(self._c[i]*point)**i
        return y


class ControllerNode:
    
    def __init__(self,initial_velocity=(0.0,0.0)):
        self.__wrapper__ = NodeWrapper('controller')
        self.__publisher__ = self.__wrapper__.create_publisher('vel.out')
        STORE.set('controller','velocity',initial_velocity)

    def control_velocity(self,velocity,num_steps=1,step_interval=1,critical=False):
        initial = STORE.get('controller','velocity')
        Dlin = velocity[0]-initial[0]
        Dang = velocity[1]-initial[1]
        dlin,dang = Dlin/num_steps, Dang/num_steps
        for _ in range(num_steps):
            if not critical and STORE.get('critical','interrupt'):
                return True
            initial = (initial[0] + dlin, initial[1] + dang)
            self.__publisher__.publish(Auxiliary.velocity_to_message(initial[0], initial[1]))
            STORE.set('controller','velocity',initial)
            Auxiliary.wait(step_interval)
    
    def stop(self,num_steps=1,step_interval=1):
        self.control_velocity((0,0),num_steps,step_interval)
    
    def rotate_by(self, angle):
        
        def yaw():
            return STORE.get('estimator','position')[2]
        
        def diff():
            return Auxiliary.normalize_angle(t1-yaw())
        
        def initial_accelerate():
            _,vlin = STORE.get('controller','velocity')
            return self.control_velocity((vlin,w*Auxiliary.sgn(initial_diff)),n,s)
        
        def midway_monitor():
            interval = 0.01
            while abs(diff()) >= resolution:
                if STORE.get('critical','interrupt'):
                    return True
                Auxiliary.wait(interval)
        
        def final_decelerate():
            _,vlin = STORE.get('controller','velocity')
            return self.control_velocity((vlin,0),n,s)

        t1 = Auxiliary.normalize_angle(yaw() + angle)
        w,n,s = 0.2,1,0
        resolution = pi/180
        initial_diff = diff()
        if abs(initial_diff) < resolution: return
        if initial_accelerate() or midway_monitor() or final_decelerate() or STORE.get('critical','interrupt'): return True

    def rotate_to(self,yaw):
        _,_,t = STORE.get('estimator','position')
        return self.rotate_by(yaw-t)

    def move_to(self, point):

        def pos():
            return STORE.get('estimator','position')
        
        def diff():
            x0,y0,t0 = pos()
            x1,y1 = point
            dx,dy = x1-x0,y1-y0
            r = Auxiliary.normalize_angle(atan2(dy,dx)-t0)
            d = sqrt(dx*dx+dy*dy)
            return d,r
        
        def reorient():
            r,t = diff()
            return self.rotate_by(t)

        def select_speed():
            s = 0.1
            f = 0.5
            c1,c2,c3,c4 = 0.12,1.384,0.05,0.5
            r,t = diff()
            if r == 0: return None
            for i in range(10,-1,-1):
                v = i*c3
                n = floor(i*c4)
                c = c1*v+c2
                dmin = c*n*s*v
                if dmin <= f*r:
                    break
            if dmin == 0: return None
            return v,n,s,dmin
        
        def initial_accelerate():
            return self.control_velocity((v,0),n,s)

        def midway_move():
            interval = 0.1
            while True:
                if STORE.get('critical','interrupt'):
                    return True
                r,t = diff()
                if r <= dmin/2:
                    break
                if abs(t) >= 10*pi/180:
                    if final_decelerate(): return True
                    return self.move_to(point)
                Auxiliary.wait(interval)

        def final_decelerate():
            return self.control_velocity((0,0),n,s)
        
        resolution = 0.1
        r0,t0 = diff()
        if r0 < resolution: return
        data = select_speed()
        if data is None: return
        v,n,s,dmin = data

        if reorient() or initial_accelerate() or midway_move() or final_decelerate() or STORE.get('critical','interrupt'): return True

    def destroy(self):
        self.__wrapper__.destroy()
    

class EstimatorNode:

    def __init__(self):
        self.__wrapper__ = NodeWrapper('estimator')
        self.__wrapper__.create_subscription('odom.in',self.__odometry_callback__)
    
    def __odometry_callback__(self, msg: Odometry):                
        roll, pitch, yaw = Auxiliary.euler_from_quaternion(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
        STORE.set('estimator','position',(msg.pose.pose.position.x,msg.pose.pose.position.y,yaw))

    def destroy(self):
        self.__wrapper__.destroy()


class SensorNode:

    def __init__(self):
        self.__wrapper__ = NodeWrapper('sensor')
        self.__wrapper__.create_subscription('laser.in',self.__laser_callback__)
    
    def __laser_callback__(self, msg: LaserScan):
        if STORE.has('estimator','position'):
            STORE.set('sensor','patch',Patch(msg,STORE.get('estimator','position')))

    def destroy(self):
        self.__wrapper__.destroy()


class Mapper:

    def __init__(self,merge_interval=1,sensor_radius=10.0,resolution=0.3,obstacle_lookup_radius=0.3,critical_radius=0.3):
        self.__threadw__ = ThreadWrapper(self.__callback__,merge_interval)
        map = Map(sensor_radius,resolution,obstacle_lookup_radius)
        cmap = CriticalMap(map,critical_radius)
        STORE.set('mapper','map',map)
        STORE.set('mapper','cmap',cmap)
        self.__threadw__.start()
    
    def __callback__(self):
        if STORE.has('sensor','patch'):
            map = STORE.get('mapper','map')
            cmap = STORE.get('mapper','cmap')
            STORE.lock('mapper')
            map.merge_patch(STORE.get('sensor','patch'))
            cmap.update_critical_map()
            STORE.release('mapper')

    def destroy(self):
        self.__threadw__.stop()


class CriticalController:
    
    def __init__(self,controller: ControllerNode,critical_interval=0.05,critical_radius=1.0):
        STORE.set('critical','interrupt',False)
        self.__threadw__ = ThreadWrapper(self.__critical_callback__,critical_interval)
        self.__controller__ = controller
        self.__threadw__.start()
        self.__critical_radius__ = critical_radius
    
    def __critical_callback__(self):
        if not STORE.has('sensor','patch'): return
        patch = STORE.get('sensor','patch')
        for dist in patch.__obstacles__:
            if dist <= self.__critical_radius__:
                self.__critical_action__()
                break

    def __critical_action__(self):
        file_out('interrupt')
        STORE.set('critical','interrupt',True)
        self.__controller__.control_velocity((0.0,0.0),4,0.05,True)
        STORE.set('critical','interrupt',False) 

    def destroy(self):
        self.__threadw__.stop()


class LocalPlanner:

    def __init__(self,critical_node:CriticalController,plan_interval=0.1):
        STORE.set('planner','plan',([],[]))
        self.__tw__ = ThreadWrapper(self.__callback__,plan_interval)
        self.__tw__.start()
        self.__cc__ = critical_node
    
    def destroy(self):
        self.__tw__.stop()

    def __neighbour_dist__(self,cell,neighbour):
        if cell[0]-neighbour[0] and cell[1]-neighbour[1]: return sqrt(2)
        return 1
    
    def __angle_diff__(self,parents_parent,parent,child):
        diff = (parent[0]-parents_parent[0])*(child[0]-parent[0])+(parent[1]-parents_parent[1])*(child[1]-parent[1])
        return acos(diff/(self.__neighbour_dist__(parents_parent,parent)*self.__neighbour_dist__(parent,child)))

    def __movement_cost__(self,parent_cell,child_cell,parent_map):
        cost_per_radian = 10
        parents_parent_cell = parent_map[parent_cell]
        ang = 0
        if parents_parent_cell is not None:
            ang = self.__angle_diff__(parents_parent_cell,parent_cell,child_cell)
        return cost_per_radian*ang + self.__heuristic_cost__(parent_cell,child_cell)
    
    def __heuristic_cost__(self,start_cell,goal_cell):
        cost_per_meter = 1
        dx = start_cell[0]-goal_cell[0]
        dy = start_cell[1]-goal_cell[1]
        return cost_per_meter*sqrt(dx*dx+dy*dy)

    def __find_path__(self,start_cell,goal_cell,obstacles):
        rows,cols = obstacles.shape
        
        if start_cell[0] < 0 or start_cell[0] >= rows or start_cell[1] < 0 or start_cell[1] >= cols:
            return []
        if goal_cell[0] < 0 or goal_cell[0] >= rows or goal_cell[1] < 0 or goal_cell[1] >= cols:
            return []
        if obstacles[start_cell[0]][start_cell[1]] == 1 or obstacles[goal_cell[0]][goal_cell[1]] == 1:
            return []
        
        frontier = PriorityQueue()
        explored = set()
        cost_map = dict()
        parent_map = dict()

        initial_cost = self.__heuristic_cost__(start_cell,goal_cell)
        frontier.put((initial_cost,start_cell))
        cost_map[start_cell] = (initial_cost,0)
        parent_map[start_cell] = None

        while not frontier.empty():
            f_cost,(i,j) = frontier.get_nowait()
            f_cost,g_cost = cost_map[(i,j)]
            explored.add((i,j))

            if (i,j) == goal_cell:
                cell = goal_cell
                path = []
                while cell is not None:
                    path.append(cell)
                    cell = parent_map[cell]
                path.reverse()
                return path
            
            for x,y in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i-1,j-1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]:
                if x >= 0 and y >= 0 and x < rows and y < cols and (x,y) not in explored and obstacles[x,y] != 1:
                    g = g_cost + self.__movement_cost__((i,j),(x,y),parent_map)
                    if (x,y) not in cost_map or cost_map[(x,y)][1] > g:
                        f = self.__heuristic_cost__((x,y),goal_cell) + g
                        frontier.put((f,(x,y)))
                        cost_map[(x,y)] = (f,g)
                        parent_map[(x,y)] = (i,j)
        
        return []
    
    def __reduce_path__(self,path):
        if len(path) < 3: return path
        reduced_path = [path[0]]
        old_direction = (path[1][0]-path[0][0],path[1][1]-path[0][1])
        for i in range(2,len(path)):
            new_direction = (path[i][0]-path[i-1][0],path[i][1]-path[i-1][1])
            if old_direction != new_direction:
                reduced_path.append(path[i-1])
                old_direction = new_direction
        reduced_path.append(path[-1])
        return reduced_path

    def __analyze_plan__(self,plan_tf,cmap):
        plan_tf = STORE.get('planner','plan')[1]
        if not plan_tf or len(plan_tf) < 2: return 'NO PLAN'
        map = cmap.get_critical_map()
        for i in range(1,len(plan_tf)):
            parent_cell = cmap.itf(plan_tf[i-1][0],plan_tf[i-1][1])
            child_cell = cmap.itf(plan_tf[i][0],plan_tf[i][1])
            di,dj = 0,0
            if parent_cell[0] < child_cell[0]: di = 1
            elif parent_cell[0] > child_cell[0]: di = -1
            if parent_cell[1] < child_cell[1]: dj = 1
            elif parent_cell[1] > child_cell[1]: dj = -1 
            k,l = parent_cell
            while (k,l) != child_cell:
                if map[k,l] == 1:
                    return 'BLOCKED'
                k += di
                l += dj
            if map[k,l] == 1: return 'BLOCKED'
        return 'UNBLOCKED'
    
    def __callback__(self):
        if STORE.get('mapper','cmap').get_critical_map() is None or not STORE.has('general','goal'): return
        goal_point = STORE.get('general','goal')
        start_point = STORE.get('estimator','position')
        STORE.lock('mapper')
        cmap = STORE.unsafe_get('mapper','cmap')
        # analysis = self.__analyze_plan__(STORE.get('planner','plan')[1],cmap)
        # if analysis == 'UNBLOCKED': return
        # elif analysis == 'BLOCKED':
        #     self.__cc__.__critical_action__()
        start = cmap.itf(start_point[0],start_point[1])
        goal = cmap.itf(goal_point[0],goal_point[1])
        path = self.__reduce_path__( self.__find_path__(start,goal,cmap.get_critical_map()))
        if path != STORE.get('planner','plan')[0]: self.__cc__.__critical_action__()
        tf_path = []
        for i,j in path:
            tf_path.append(cmap.tf(i,j))
        STORE.release('mapper')
        STORE.set('planner','plan',(path,tf_path))
      

def cv2_tf(pt):
    return pt[1],pt[0]

def visualize_map(name,resize=None):

    cmap = STORE.get('mapper','cmap')
    matrix = cmap.get_critical_map()
    if matrix is None: return

    cx,cy,_ = STORE.get('estimator','position')

    STORE.lock('mapper')
    
    row,col = matrix.shape
    viz = np.full((row,col,3),128,np.uint8)
    for i in range(row):
        for j in range(col):
            if matrix[i,j] == 1:
                viz[i,j] = np.array([0,0,0],np.uint8)
            elif matrix[i,j] == 0:
                viz[i,j] = np.array([255,255,255],np.uint8)

    current_cell = cmap.itf(cx,cy)

    STORE.release('mapper')

    viz = cv2.circle(viz,cv2_tf(current_cell),1,(0,0,255),cv2.FILLED)

    if STORE.has('planner','plan'):
        plan,tf_plan = STORE.get('planner','plan')
        if plan:
            for k in range(1,len(plan)):
                viz = cv2.line(viz,cv2_tf(plan[k-1]),cv2_tf(plan[k]),(255,0,0),1)
            viz = cv2.circle(viz,cv2_tf(plan[0]),1,(0,255,255),cv2.FILLED)
            viz = cv2.circle(viz,cv2_tf(plan[-1]),1,(0,255,0),cv2.FILLED)

    if resize is not None:
        viz = cv2.resize(viz,resize)
    
    cv2.imshow(name,viz)

f = open('/home/ghost/Desktop/my_working','w')

def file_out(x):
    f.write(str(x))
    f.write('\n')
    f.flush()

def file_in():
    f2 = open('/home/ghost/Desktop/my_input','r')
    lines = f2.read().splitlines()
    return lines

def main(args=None):
    global EXECUTOR
    rclpy.init()

    EXECUTOR = MultiThreadedExecutor(os.cpu_count())
    estimator = EstimatorNode()
    sensor = SensorNode()
    controller = ControllerNode()
    mapper = Mapper(critical_radius=5,merge_interval=0.1)
    critical_controller = CriticalController(controller)
    
    spin_thread = Thread(target=EXECUTOR.spin)
    spin_thread.start()

    planner = LocalPlanner(critical_controller)

    def mover():
        if not STORE.has('estimator','position'): return
        plan,plan_tf = STORE.get('planner','plan')
        position = STORE.get('estimator','position')
        goal = None
        for cell in plan_tf:
            if Auxiliary.cartesian_dist(cell,position) >= 0.5:
                goal = cell
                break
        if goal is not None:
            controller.move_to(goal)
    
    def viz():
        visualize_map('Map',(800,800))
        cv2.waitKey(100)
    
    move_thread = ThreadWrapper(mover,0.1)
    viz_thread = ThreadWrapper(viz,0.1)

    move_thread.start()
    viz_thread.start()

    while True:
        try:
            x,y = (float(_) for _ in file_in()[0].split(','))
            STORE.set('general','goal',(x,y))
        except:
            break
        sleep(0.1)

    planner.destroy()
    viz_thread.stop()
    move_thread.stop()
    EXECUTOR.shutdown()
    spin_thread.join()
    critical_controller.destroy()
    mapper.destroy()
    controller.destroy()
    estimator.destroy()
    sensor.destroy()
    rclpy.shutdown()
    cv2.destroyAllWindows()

    f.close()