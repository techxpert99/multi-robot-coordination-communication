from logging import critical
from math import asin, acos, atan2, ceil, cos, exp, floor, log, sin, sqrt, pi
from queue import PriorityQueue
from sys import stdout
from time import time

from sympy import hyper
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data
from os import makedirs,remove
from os.path import isdir,isfile
import numpy as np
from collections import OrderedDict
from sensor_msgs.msg import LaserScan
import cv2
import rclpy

# Deprecated Constants

# obstacle_exclude_distance = 0.4
# num_skip_poses = 10
# replanning_period = 50000
# obstacle_expansion = ceil((low_visibility_lower_bound+robot_width)/resolution)
# visibility_displacement = 0.1



# Independent Constants

float_precision=0.0000000001

region_data_length = 500
region_memory_cache_size=16
region_cache_size=128

obstacle_identifier = 0
no_visibility_identifier = 1
low_visibility_identifier = 2
high_visibility_identifier = 3

cache_flush_state = 1200000
lidar_sensor_patch_state = 2
visualization_state = 100
destroyed_state = 4
num_buffer_transitions_to_stop = 60

plan_line_thickness = 3
plan_point_radius = 3

resolution = 0.2
sensor_range = 7.0
robot_width = 0.2

low_visibility_lower_bound = 0.2
high_visibility_lower_bound = 1.0

stuck_radius = 0.05
unlodge_radius = 1.0

barrier_probability = 0.2
occupancy_probablity_tolerance = 0.2
gaussian_spread = 1.

lower_linear_tolerance=0.4
upper_linear_tolerance=0.6
lower_angular_tolerance=3*pi/180
upper_angular_tolerance=6*pi/180
lower_reorientation_tolerance=6*pi/180
upper_reorientation_tolerance=20*pi/180

linear_smoothing=40.
angular_smoothing=1.
maximum_linear_speed = 0.75
maximum_slow_linear_speed = 0.375
maximum_angular_speed = 0.5
maximum_slow_angular_speed = 0.25

low_visibility_cell_traversal_penalty = 100.
angular_reorientation_penalty = 5.

region_cache_location='cached_regions/'


# Dependent Constants

two_pi = 2*pi
region_length = region_data_length*resolution
half_resolution = resolution/2.
half_robot_width = robot_width/2.

critical_radius = ceil((robot_width/sqrt(2)+low_visibility_lower_bound)/resolution)
hyper_critical_radius = ceil(sqrt(2)*(robot_width/sqrt(2)+high_visibility_lower_bound)/resolution)

gaussian_peak_probability = 1-barrier_probability
gaussian_base_probability = barrier_probability

gaussian_rising_interval = sqrt(gaussian_spread*log(gaussian_peak_probability/gaussian_base_probability))
gaussian_falling_interval = sqrt(gaussian_spread*log(2*gaussian_peak_probability))

peak_occupancy = log(gaussian_peak_probability/(1-gaussian_peak_probability))
base_occupancy = log(gaussian_base_probability/(1-gaussian_base_probability))

occupancy_lower_threshold = log((0.5+occupancy_probablity_tolerance/2)/(0.5-occupancy_probablity_tolerance/2))
unoccupancy_upper_threshold = log((0.5-occupancy_probablity_tolerance/2)/(0.5+occupancy_probablity_tolerance/2))


# State Variables

average_fps = 0
num_calls = 0


# Code

def EuclideanDistance(source, target):
    dx = source[0]-target[0]
    dy = source[1]-target[1]
    return sqrt(dx*dx+dy*dy)

def PositiveAngle(angle):
	return (angle+2*pi)%(2*pi)

def AngleToTarget(source,target):
    t0 = (atan2(target[1]-source[1],target[0]-source[0])-source[2]+two_pi)%two_pi
    t1 = two_pi-t0
    return t0 if t0 <= t1 else -t1

def ReoirentationAngleToTarget(source,target):
    t0 = (target[2]-source[2]+two_pi)%two_pi
    t1 = two_pi-t0
    return t0 if t0 <= t1 else -t0

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
    roll,pitch,yaw = TransformQuaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    return (msg.pose.pose.position.x,msg.pose.pose.position.y,yaw)

# Coalesces the plan
def CoalescePlan(plan):
    if len(plan) < 3: return plan
    coalesced_plan = [plan[0]]
    old_direction = (plan[1][0]-plan[0][0]+float_precision,plan[1][1]-plan[0][1]+float_precision)
    mod = EuclideanDistance((0,0),old_direction)
    old_direction = old_direction[0]/mod,old_direction[1]/mod
    for i in range(2,len(plan)):
        new_direction = (plan[i][0]-plan[i-1][0]+float_precision,plan[i][1]-plan[i-1][1]+float_precision)
        mod = EuclideanDistance((0,0),new_direction)
        new_direction = new_direction[0]/mod,new_direction[1]/mod
        if abs(old_direction[0]-new_direction[0]) >= float_precision or abs(old_direction[1]-new_direction[1]) >= float_precision:
            coalesced_plan.append(plan[i-1])
            old_direction = new_direction
    coalesced_plan.append(plan[-1])
    return coalesced_plan

def AccelerateAngular(cangular,slow=False):
    angular = maximum_slow_angular_speed if slow else maximum_angular_speed
    if cangular > 0.:
        return min(angular,cangular+angular/angular_smoothing)
    return max(-angular,cangular-angular/angular_smoothing)

def DecelerateAngular(cangular):
    if cangular > 0:
        return max(0.,cangular-maximum_angular_speed/angular_smoothing)
    return min(0.,cangular+maximum_angular_speed/angular_smoothing)

def AccelerateLinear(clinear,slow=False):
    linear = maximum_slow_linear_speed if slow else maximum_linear_speed
    return min(linear,clinear+linear/linear_smoothing)

def DecelerateLinear(clinear):
    return max(0.,clinear-maximum_linear_speed/linear_smoothing)

# Reduces the plan so that the robot does not go back and forth
def ReducePlan(position,plan):
    px,py,_ = position
    cache = None
    for i in range(1,len(plan)):
        ax,ay = plan[i-1]
        bx,by = plan[i]
        if bx == ax:
            entry = (abs(ax-px),(ax,by),i)
        else:
            m = (by-ay)/(bx-ax)
            x = (m*(py-ay)+m*m*ax+px)/(1+m*m)
            if x >= min(ax,bx) and x <= max(ax,bx):
                y = m*(x-ax)+ay
                d = EuclideanDistance((px,py),(x,y))
                entry = (d,(x,y),i)
            else:
                da = EuclideanDistance((px,py),(ax,ay))
                db = EuclideanDistance((px,py),(bx,by))
                if da < db:
                    entry = (da,(ax,ay),i)
                else:
                    entry = (db,(bx,by),i)
        if cache is None or entry[0] <= cache[0]+float_precision:
            cache = entry
    plan = plan[cache[2]:]
    plan.insert(0,cache[1])
    return plan

class RegionCache:
    
    def __init__(self):
        if not isdir(region_cache_location):
            makedirs(region_cache_location)
        self.cached_regions = OrderedDict()
        self.regions_in_memory = OrderedDict()

    # Determines if a region is cached
    def HasRegion(self,region_id):
        return region_id in self.cached_regions
    
    # Determines if a region is cached in memory
    def HasRegionInMemory(self,region_id):
        return region_id in self.regions_in_memory
    
    # Inserts a region into the cache
    def InsertRegion(self,region_id,region_data):
        if region_id in self.regions_in_memory:
            del self.cached_regions[region_id]
            del self.regions_in_memory[region_id]
        elif region_id in self.cached_regions:
            del self.cached_regions[region_id]
        else:
            if len(self.cached_regions) == region_cache_size:
                if region_cache_size > region_memory_cache_size:
                    delete_region_id = self.cached_regions.popitem(False)[0]
                    remove(region_cache_location+'/'+str(delete_region_id[0])+','+str(delete_region_id[1])+'.npy')
                    remove(region_cache_location+'/p'+str(delete_region_id[0])+','+str(delete_region_id[1])+'.npy')
                    evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)
                    if self.cached_regions[evict_region_id]:
                        np.save(region_cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data[0],allow_pickle=False)
                        np.save(region_cache_location+'/p'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data[1],allow_pickle=False)
                else:
                    self.cached_regions.popitem(False)
                    self.regions_in_memory.popitem(False)
            elif len(self.regions_in_memory) == region_memory_cache_size:
                evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)
                if self.cached_regions[evict_region_id]:
                    np.save(region_cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data[0],allow_pickle=False)
                    np.save(region_cache_location+'/p'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data[1],allow_pickle=False)
        self.cached_regions[region_id] = True
        self.regions_in_memory[region_id] = region_data

    # Gets a region
    def GetRegion(self,region_id):
        if region_id not in self.cached_regions:
            return
        elif region_id in self.regions_in_memory:
            modified = self.cached_regions[region_id]
            region_data = self.regions_in_memory[region_id]
            del self.regions_in_memory[region_id]
        else:
            if len(self.regions_in_memory) == region_memory_cache_size:
                evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)
                if self.cached_regions[evict_region_id]:
                    np.save(region_cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data[0],allow_pickle=False)
                    np.save(region_cache_location+'/p'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data[1],allow_pickle=False)
            region_data = np.load(region_cache_location+'/'+str(region_id[0])+','+str(region_id[1])+'.npy',None,False),np.load(region_cache_location+'/p'+str(region_id[0])+','+str(region_id[1])+'.npy',None,False)
            modified = False
        del self.cached_regions[region_id]
        self.cached_regions[region_id] = modified
        self.regions_in_memory[region_id] = region_data
        return region_data

    # Loads the cache with pre-saved information
    def InitialLoad(self):
        if not isfile(region_cache_location+'/meta.npy'):
            return
        region_ids = np.load(region_cache_location+'/meta.npy',None,False)
        num_loads = min(region_cache_size-len(self.regions_in_memory),len(region_ids))
        for i in range(num_loads):
            region_id = tuple(region_ids[num_loads-1-i])
            self.regions_in_memory[region_id] = np.load(region_cache_location+'/'+str(region_id[0])+','+str(region_id[1])+'.npy',None,False),np.load(region_cache_location+'/p'+str(region_id[0])+','+str(region_id[1])+'.npy',None,False)
        for i in range(len(region_ids)):
            self.cached_regions[tuple(region_ids[len(region_ids)-1-i])] = False
    
    # Flushes the contents of the cache to the disk
    def Flush(self):
        for region_id in self.regions_in_memory:
            if self.cached_regions[region_id]:
                region_data = self.regions_in_memory[region_id]
                np.save(region_cache_location+'/'+str(region_id[0])+','+str(region_id[1]),region_data[0],allow_pickle=False)
                np.save(region_cache_location+'/p'+str(region_id[0])+','+str(region_id[1]),region_data[1],allow_pickle=False)
        region_ids = np.zeros((len(self.cached_regions),2),np.int32)
        i = 0
        for region_id in self.cached_regions:
            region_ids[i] = region_id
            i += 1
        np.save(region_cache_location+'/meta',region_ids,allow_pickle=False)
        

# Region denotations:
# Obstacles: 0
# Obstacle-free cell: 100
# Unexplored area: 200

class World:

    def __init__(self):

        #Region Cache
        self.cache = RegionCache()
        print('Loading cache')
        self.cache.InitialLoad()

    # Transforms a position (x,y) to a region id (a,b)
    def TransformPositionToRegion(self,position):
        return floor(position[0]/region_length),floor(position[1]/region_length)
    
    # Transforms a position (x,y) to local coordinates (i,j)
    def TransformPositionToLocalCoordinates(self,position):
        return floor(position[1]/resolution)-region_data_length*floor(position[1]/region_length),floor(position[0]/resolution)-region_data_length*floor(position[0]/region_length)

    # Transforms local coordinates to position    
    def TransformLocalCoordinatesToPosition(self,region_id,local_coordinates):
        return (local_coordinates[1]+0.5)*resolution+region_length*region_id[0],(local_coordinates[0]+0.5)*resolution+region_length*region_id[1]

    def GetRegion(self,region_id):
        region_data = None
        # Check if the source region is cached. If it is get it, otherwise create a new region, cache it and get it
        if self.cache.HasRegion(region_id):
            region_data = self.cache.GetRegion(region_id)
        else:
            region_data = np.full((region_data_length,region_data_length),0.,np.float32),np.full((region_data_length,region_data_length),high_visibility_identifier,np.uint8)
            self.cache.InsertRegion(region_id,region_data)
        return region_data

    def SetRegion(self,region_id,region_data):
        self.cache.InsertRegion(region_id,region_data)

    # Plans a path from a source position (x1,y1) to a target position (x2,y2)
    def PlanPath(self,source_position,target_position):

        # Try local planning first. If it fails, switch to global planning
        local_plan = self.LocalPlanPath(source_position,target_position)
        if local_plan is not None: return local_plan

        # Try global planning


    def LocalPlanPath(self,source_position,target_position):
        
        source_region_id = self.TransformPositionToRegion(source_position)
        target_region_id = self.TransformPositionToRegion(target_position)
        
        # If the source position and the target position are in different regions, local planning cannot proceed
        if source_region_id != target_region_id: return None
        
        source_cell = self.TransformPositionToLocalCoordinates(source_position)
        target_cell = self.TransformPositionToLocalCoordinates(target_position)
        region_data = self.GetRegion(source_region_id)[1]

        if region_data[target_cell[0]][target_cell[1]] == obstacle_identifier: return None

        frontier = PriorityQueue()
        explored = set()
        gcosts = dict()
        parents = dict()

        frontier.put((EuclideanDistance(source_cell,target_cell),source_cell))
        gcosts[source_cell] = 0
        parents[source_cell] = None

        num_rows,num_cols = region_data.shape

        while not frontier.empty():

            f,cell = frontier.get()
            g = gcosts[cell]
            explored.add(cell)

            if cell == target_cell:
                intermediate_cell = target_cell
                path = []
                while intermediate_cell is not None:
                    path.append(self.TransformLocalCoordinatesToPosition(source_region_id,intermediate_cell))
                    intermediate_cell = parents[intermediate_cell]
                path.reverse()
                return path

            i,j = cell
            parent = parents[cell]
            if parent is not None: parent_distance = EuclideanDistance(cell,parent)

            for i2,j2 in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i-1,j-1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]:
                child_cell = (i2,j2)
                if i2 >= 0 and j2 >= 0 and i2 < num_rows and j2 < num_cols and child_cell not in explored:
                    # if EuclideanDistance(source_cell,child_cell)*resolution <= obstacle_exclude_distance or region_data[child_cell] != obstacle_identifier:
                    data = region_data[child_cell]
                    if data != obstacle_identifier and data != no_visibility_identifier:
                        gchild = g + EuclideanDistance(cell,child_cell)
                        if data == low_visibility_identifier:
                            gchild += low_visibility_cell_traversal_penalty
                        if parent is not None: 
                            gchild += angular_reorientation_penalty*acos(((cell[0]-parent[0])*(child_cell[0]-cell[0])+(cell[1]-parent[1])*(child_cell[1]-cell[1]))/(EuclideanDistance(child_cell,cell)*parent_distance))
                        if child_cell not in gcosts or gcosts[child_cell] > gchild:
                            fchild = gchild + EuclideanDistance(child_cell,target_cell)
                            frontier.put((fchild,child_cell))
                            gcosts[child_cell] = gchild
                            parents[child_cell] = cell
        
        return None

    def GetAtPosition(self,position):
        region_id = self.TransformPositionToRegion(position)
        local_coordinates = self.TransformPositionToLocalCoordinates(position)
        return self.GetRegion(region_id)[0][local_coordinates]
    
    def RegionBounds(self,region_id):
        return (region_id[0]*region_length,region_id[1]*region_length,(region_id[0]+1)*region_length,(region_id[1]+1)*region_length)


class Controller:
    
    def __init__(self, parameter_store, robot_namespace):
        self.node = Node('controller',namespace=robot_namespace)
        self.parameter_store = parameter_store
        
        self.velocity_publisher = self.node.create_publisher(Twist,robot_namespace+'/velocity',qos_profile_system_default)
        self.odometry_subscription = self.node.create_subscription(Odometry,robot_namespace+'/odometery',self.EstimatePosition,qos_profile_sensor_data)
        self.lidar_subscription = self.node.create_subscription(LaserScan,robot_namespace+'/laser/out',self.PatchMapWithSensorData,qos_profile_sensor_data)
        self.world = World()

        self.position = None
        self.plan = None
        self.target = None
        self.slow_chase = False
        self.goal = None
        self.velocity = Twist()

        self.velocity_controller_state = 0
        self.plan_controller_state = 0
        self.sensor_controller_state = 0
        self.visualizer_controller_state = 0
        self.cache_flush_controller_state = 0
        self.destruction_controller_state = 0

        self.prev_vel = 0.
    
    def EstimatePosition(self, msg: Odometry):
        self.position = UnmarshalPosition(msg)

    def TransformRobotCoordinatesToPosition(self,radial_distance,angle):
        return self.position[0]+radial_distance*sin(self.position[2]+angle),self.position[1]-radial_distance*cos(self.position[2]+angle)

    def FillObstacleVicinities(self,obstacle_data,vicinity_data,min_cell,max_cell):
        
        # Dependent Parameters

        k1,l1 = min_cell
        k2,l2 = max_cell
        M,N = obstacle_data.shape
        n = l2-l1+1

        # Micro-optimization

        c1 = k1-2*hyper_critical_radius
        c2 = k2+2*hyper_critical_radius+1
        c3 = max(0,l1-2*hyper_critical_radius)
        c4 = min(N,l2+2*hyper_critical_radius+1)
        c5 = -hyper_critical_radius
        c6 = -l1+4*hyper_critical_radius+1
        c7 = k1-2*critical_radius
        c8 = k2+2*critical_radius
        c9 = l1-2*critical_radius
        c10 = l2+2*critical_radius
        c11 = -critical_radius
        c12 = -l1+4*critical_radius+1
        c13 = 0
        c14 = -l1+3*hyper_critical_radius
        c15 = max(0,l1-hyper_critical_radius)
        c16 = min(N,l2+hyper_critical_radius+1)
        c17 = 2*hyper_critical_radius+1
        c18 = 0
        c19 = -l1+3*critical_radius
        c20 = max(0,l1-critical_radius)
        c21 = l2+critical_radius+1
        c22 = 2*critical_radius+1
        c23 = n+4*hyper_critical_radius
        c24 = 0
        c25 = 0
        c27 = 0
        c28 = n+4*critical_radius
        c29 = 0
        c30 = 0
        c31 = -l1+2*hyper_critical_radius
        c32 = -l1+2*critical_radius
        c33 = critical_radius
        c34 = hyper_critical_radius
        c35 = M+hyper_critical_radius
        c36 = M+critical_radius
        c37 = max(-l1+3*hyper_critical_radius,2*hyper_critical_radius)
        c38 = obstacle_identifier
        c39 = no_visibility_identifier
        c40 = high_visibility_identifier
        c41 = low_visibility_identifier
        c42 = occupancy_lower_threshold
        c43 = max(-l1+3*critical_radius,2*critical_radius)
        c44 = -max(0,l1-hyper_critical_radius)
        c45 = -max(0,l1-critical_radius)
        c46 = 0
        c47 = 0

        # Caches

        t1 = [0]*(n+2*hyper_critical_radius)
        t2 = [0]*(n+2*critical_radius)
        t3 = [0]*(n+6*hyper_critical_radius+1)
        t4 = [0]*(n+6*critical_radius+1)

        # Data

        x1 = obstacle_data
        x2 = vicinity_data

        # Logic

        for k in range(c1,c2):
            c24 = k >= c7 and k <= c8
            c25 = k >= k1
            c26 = k >= 0 and k < M
            c29 = c25 and k >= c34 and k < c35
            c30 = c25 and k >= c33 and k < c36
            for l in range(c23):
                t3[l] = 0
            if c24:
                for l in range(c28):
                    t4[l] = 0
            if c26:
                c46 = x1[k]
                for l in range(c3,c4):
                    if c46[l] > c42:
                        t3[l+c31] += 1
                        t3[l+c6] -= 1
                        if c24 and l >= c9 and l <= c10:
                            t4[l+c32] += 1
                            t4[l+c12] -= 1
            c13 = 0
            c46 = x2[k+c5]
            for l in range(c37):
                c13 += t3[l]
            for l in range(c15,c16):
                c13 += t3[l+c14]
                if c13:
                    t1[l+c44] = c17
                if t1[l+c44]:
                    t1[l+c44] -= 1
                    if c29:
                        c27 = c46[l]
                        if c27 != c38 and c27 != c39:
                            c46[l] = c41
                elif c29:
                    c46[l] = c40
            if c24:
                c18 = 0
                c46 = x1[k+c11]
                c47 = x2[k+c11]
                for l in range(c43):
                    c18 += t4[l]
                for l in range(c20,c21):
                    c18 += t4[l+c19]
                    if c18:
                        t2[l+c45] = c22
                    if t2[l+c45]:
                        t2[l+c45] -= 1
                        if c30:
                            if c46[l] > c42:
                                c47[l] = c38
                            else:
                                c47[l] = c39

    def FillObstacles(self,sensor_message,obstacle_data,region_id):
        
        sensor_data = sensor_message.ranges
        dt = sensor_message.angle_increment
        dr = resolution
        t=0
        for z in sensor_data:
            r_gaussian_begin = z-gaussian_rising_interval
            r_end = min(sensor_range,z+gaussian_falling_interval)
            r = 0
            while r <= r_end:
                pose = self.TransformRobotCoordinatesToPosition(r,t)
                if region_id == self.world.TransformPositionToRegion(pose):
                    cell = self.world.TransformPositionToLocalCoordinates(pose)
                    occupancy = obstacle_data[cell]
                    if r < r_gaussian_begin:
                        obstacle_data[cell] = base_occupancy if occupancy<=0 else base_occupancy+occupancy
                    else:
                        prob = gaussian_peak_probability*exp(-(r-z)*(r-z)/gaussian_spread)
                        delta = log((prob+float_precision)/(1-prob+float_precision))
                        obstacle_data[cell] = min(peak_occupancy,max(base_occupancy,occupancy+delta))
                r += dr
            t += dt

    def PatchMapWithSensorData(self, msg: LaserScan):

        if not self.position: return

        state = self.sensor_controller_state

        if state == lidar_sensor_patch_state:
            
            _b = time()

            min_pose = (self.position[0]-sensor_range,self.position[1]-sensor_range)
            max_pose = (self.position[0]+sensor_range,self.position[1]+sensor_range)
            i1,j1 = self.world.TransformPositionToRegion(min_pose)
            i2,j2 = self.world.TransformPositionToRegion(max_pose)

            for i in range(i1,i2+1):
                for j in range(j1,j2+1):

                    region_id = (i,j)
                    region_data = self.world.GetRegion(region_id)
                    obstacle_data,vicinity_data = region_data
                    self.FillObstacles(msg,obstacle_data,region_id)
                    region_bounds = self.world.RegionBounds(region_id)
                    begin_pose = max(min_pose[0],region_bounds[0]+float_precision),max(min_pose[1],region_bounds[1]+float_precision)
                    end_pose = min(max_pose[0],region_bounds[2]-float_precision),min(max_pose[1],region_bounds[3]-float_precision)
                    min_cell = self.world.TransformPositionToLocalCoordinates(begin_pose)
                    max_cell = self.world.TransformPositionToLocalCoordinates(end_pose)
                    self.FillObstacleVicinities(obstacle_data,vicinity_data,min_cell,max_cell)
                    self.world.SetRegion(region_id,region_data)
            
            state = 0
            
            _e = time()
            stdout.write(f'\rfps:{1/(_e-_b)}')

        else: state += 1

        self.sensor_controller_state = state
    
    def ChaseTarget(self):
        
        if not self.position:
            self.velocity.linear.x = 0.
            self.velocity.angular.z = 0.
            return
        
        if not self.target:
            self.velocity.linear.x = DecelerateLinear(self.velocity.linear.x)
            self.velocity.angular.z = DecelerateAngular(self.velocity.angular.z)
            return
        
        source = self.position
        target = self.target
        clinear = self.velocity.linear.x
        cangular = self.velocity.angular.z
        angular = cangular
        linear = clinear
        state = self.velocity_controller_state
        slow = self.slow_chase
        distance = EuclideanDistance(source,target)
        angle = AngleToTarget(source,target)
        
        if state == 1:
            if distance >= upper_linear_tolerance:
                if abs(angle) >= upper_angular_tolerance:
                    angular = float_precision if angle>0 else -float_precision
                    state = 2
                else: state = 4
            else: state = 9
        
        elif state == 2:
            if (cangular<0 and angle<=-lower_angular_tolerance) or (cangular>=0 and angle>=lower_angular_tolerance):
                angular = AccelerateAngular(cangular,slow)
            else: state = 3
        
        elif state == 3:
            if abs(cangular) >= float_precision:
                angular = DecelerateAngular(cangular)
            else:
                self.velocity_controller_buffer_state = 0
                self.velocity_controller_buffer_next_state = 1
                state = 12
        
        elif state == 4:
            if distance >= upper_linear_tolerance:
                if abs(angle) < upper_reorientation_tolerance: state = 5
                else:
                    angular = float_precision if angle>0 else -float_precision
                    state = 7
            else: state = 9
        
        elif state == 5:
            if distance >= lower_linear_tolerance and abs(angle) < upper_reorientation_tolerance:
                linear = AccelerateLinear(clinear,slow)
            else: state = 6
        
        elif state == 6:
            if clinear >= float_precision:
                linear = DecelerateLinear(clinear)
            else:
                self.velocity_controller_buffer_state = 0
                self.velocity_controller_buffer_next_state = 4
                state = 12
        
        elif state == 7:
            if (cangular<0 and angle<=-lower_reorientation_tolerance) or (cangular>=0 and angle>=lower_reorientation_tolerance):
                angular = AccelerateAngular(cangular,slow)
            else: state = 8
        
        elif state == 8:
            if abs(cangular) >= float_precision:
                angular = DecelerateAngular(cangular)
            else:
                self.velocity_controller_buffer_state = 0
                self.velocity_controller_buffer_next_state = 4
                state = 12
        
        elif state == 9:
            if target[2] is not None:
                reorient_angle = ReoirentationAngleToTarget(source,target)
                if abs(reorient_angle) >= upper_angular_tolerance:
                    angular = float_precision if reorient_angle>0 else -float_precision
                    state = 10
                else: state = 0
            else: state = 0
        
        elif state == 10:
            reorient_angle = ReoirentationAngleToTarget(source,target)
            if (cangular<0 and angle<=-lower_angular_tolerance) or (cangular>=0 and angle>=lower_angular_tolerance):
                angular = AccelerateAngular(cangular,slow)
            else: state = 11
        
        elif state == 11:
            if abs(cangular) >= float_precision:
                angular = DecelerateAngular(cangular)
            else:
                self.velocity_controller_buffer_state = 0
                self.velocity_controller_buffer_next_state = 9
                state = 12
        
        elif state == 12:
            if self.velocity_controller_buffer_state < num_buffer_transitions_to_stop:
                self.velocity_controller_buffer_state += 1
            else:
                state = self.velocity_controller_buffer_next_state
        
        self.velocity.linear.x = linear
        self.velocity.angular.z = angular
        self.velocity_controller_state = state

        # stdout.write(f'v:{linear:.2f},w:{angular:.2f},s:{state},d:{distance:.2f},t:{angle:.2f}\n')

    def VerifyVisibility(self,visibility_radius):
        
        xmin = self.position[0]-robot_width/sqrt(2)-visibility_radius
        ymin = self.position[1]-robot_width/sqrt(2)-visibility_radius
        xmax = self.position[0]+robot_width/sqrt(2)+visibility_radius
        ymax = self.position[1]+robot_width/sqrt(2)+visibility_radius

        i1,j1 = self.world.TransformPositionToRegion((xmin,ymin))
        i2,j2 = self.world.TransformPositionToRegion((xmax,ymax))

        for i in range(i1,i2+1):
            for j in range(j1,j2+1):
                
                region_id = (i,j)
                region_data = self.world.GetRegion(region_id)[0]
                region_bounds = self.world.RegionBounds(region_id)
                umin,vmin = self.world.TransformPositionToLocalCoordinates((max(xmin,region_bounds[0]+float_precision),max(ymin,region_bounds[1]+float_precision)))
                umax,vmax = self.world.TransformPositionToLocalCoordinates((min(xmax,region_bounds[2]-float_precision),min(ymax,region_bounds[3]-float_precision)))
                
                for u in range(umin,umax+1):
                    for v in range(vmin,vmax+1):
                        if region_data[u,v] > occupancy_lower_threshold:
                            return False
        
        return True
    
    def DecideTarget(self):

        pose = self.plan.pop(0)
        self.target = pose[0],pose[1],None
        self.velocity_controller_state = 1
        
        print('Target Set:',pose)
    
    def PlanForGoal(self):

        if not self.position or not self.goal: return

        position = self.position
        goal = self.goal
        state = self.plan_controller_state

        if state == 1:
            self.target = None
            self.velocity_controller_state = 0
            self.plan_controller_buffer_state = 0
            self.plan_controller_buffer_next_state = 3
            state = 2
        
        elif state == 2:
            if self.plan_controller_buffer_state < num_buffer_transitions_to_stop:
                self.plan_controller_buffer_state += 1
            else: state = self.plan_controller_buffer_next_state
        
        elif state == 3:
            visibility = self.VerifyVisibility(low_visibility_lower_bound)
            if visibility: plan = self.world.PlanPath(position,goal)
            if not visibility or not plan or len(plan) < 2:
                if not visibility:
                    self.visualizer_controller_state = visualization_state
                    self.VisualizeCurrentRegion()
                    print('Entered no visibility region')
                if EuclideanDistance(position,goal) < upper_linear_tolerance:
                    print('Goal Reached')
                else:
                    print('Cannot find a path to the goal')
                state = 0
            else:
                self.plan = CoalescePlan(ReducePlan(position,plan))
                state = 4
        
        elif state == 4:
            if self.plan:
                self.DecideTarget()
                state = 5
            else:
                if EuclideanDistance(position,goal) < upper_linear_tolerance:
                    print('Goal Reached')
                else:
                    print('Cannot find a path to the goal')
                state = 0
        
        elif state == 5:
            visibility = self.VerifyVisibility(high_visibility_lower_bound)
            if not visibility:
                print('Entered low visibility region')
                self.target = None
                self.velocity_controller_state = 0
                self.plan_controller_buffer_state = 0
                self.plan_controller_buffer_next_state = 6
                state = 2
            elif self.velocity_controller_state == 0: state = 4
        
        elif state == 6:                
            if EuclideanDistance(position,goal) < upper_linear_tolerance:
                print('Goal Reached')
                state = 0
            else:
                visibility = self.VerifyVisibility(low_visibility_lower_bound)
                if not visibility:
                    print('Entered no visibility region')
                    self.visualizer_controller_state = visualization_state
                    self.VisualizeCurrentRegion()
                    print('Cannot find a path to the goal')
                    state = 0
                else:
                    plan = self.world.PlanPath(position,goal)
                    if not plan or len(plan) < 2:
                        print('Cannot find a path to the goal')
                        state = 0
                    else:
                        self.plan = CoalescePlan(ReducePlan(position,plan))
                        self.slow_chase = True
                        state = 7

        elif state == 7:
            if self.plan:
                self.DecideTarget()
                state = 8
            else:
                if EuclideanDistance(position,goal) < upper_linear_tolerance:
                    print('Goal Reached')
                else:
                    print('Cannot find a path to the goal')
                self.slow_chase = False
                state = 0
        
        elif state == 8:
            visibility = self.VerifyVisibility(high_visibility_lower_bound)
            if not visibility:
                visibility = self.VerifyVisibility(low_visibility_lower_bound)
                if not visibility:
                    print('Entered no visibility region')
                    self.visualizer_controller_state = visualization_state
                    self.VisualizeCurrentRegion()
                    self.target = None
                    self.velocity_controller_state = 0
                    self.plan_controller_buffer_state = 0
                    self.plan_controller_buffer_next_state = 9
                    state = 2
                elif self.velocity_controller_state == 0: state = 7
            else:
                print('Re-entered high visibility region')
                self.slow_chase = False
                state = 5
        
        elif state == 9:
            if EuclideanDistance(position,goal) < upper_linear_tolerance:
                print('Goal Reached')
            else:
                print('Cannot find a path to the goal')
            self.slow_chase = False
            state = 0
        
        if state == 0:
            self.visualizer_controller_state = visualization_state
            self.VisualizeCurrentRegion()

        self.plan_controller_state = state
        
        # print('plan:',state)

    def FlushCache(self):

        state = self.cache_flush_controller_state

        if state == cache_flush_state:
            print('Flushing cache')
            self.world.cache.Flush()
            state = 0
        else: state += 1

        self.cache_flush_controller_state = state

    def CheckForDestruction(self):

        state = self.destruction_controller_state
        
        if state == 1:
            print('Destroying controller')
            self.target = None
            self.goal = None
            self.plan = None
            self.plan_controller_state = 0
            self.velocity_controller_state = 0
            self.destruction_controller_buffer_state = 0
            self.destruction_controller_buffer_next_state = 3
            state = 2
        
        elif state == 2:
            if state == num_buffer_transitions_to_stop:
                state = self.destruction_controller_buffer_next_state
            else: state += 1
        
        elif state == 3:
            print('Flushing Cache')
            self.world.cache.Flush()
            self.node.destroy_node()
            state = destroyed_state
        
        self.destruction_controller_state = state

    def TryDislodgeRobot(self):
        
        center = self.position

        i1,j1 = self.world.TransformPositionToRegion((center[0]-stuck_radius,center[1]-stuck_radius))
        i2,j2 = self.world.TransformPositionToRegion((center[0]+stuck_radius,center[1]+stuck_radius))
        for i in range(i1,i2+1):
            for j in range(j1,j2+1):
                region_id = (i,j)
                obstacles,_ = self.world.GetRegion(region_id)
                bounds = self.world.RegionBounds(region_id)
                ibounds = (max(bounds[0]+float_precision,center[0]-stuck_radius),max(bounds[1]+float_precision,center[1]-stuck_radius),min(bounds[2]-float_precision,center[0]+stuck_radius),min(bounds[3]+float_precision,center[1]+stuck_radius))
                k1,l1 = self.world.TransformPositionToLocalCoordinates((ibounds[0],ibounds[1]))
                k2,l2 = self.world.TransformPositionToLocalCoordinates((ibounds[2],ibounds[3]))
                for k in range(k1,k2+1):
                    row = obstacles[k]
                    for l in range(l1,l2+1):
                        if row[l] > occupancy_lower_threshold:
                            return None
        
        i1,j1 = self.world.TransformPositionToRegion((center[0]-unlodge_radius,center[1]-unlodge_radius))
        i2,j2 = self.world.TransformPositionToRegion((center[0]+unlodge_radius,center[1]+unlodge_radius))
        for i in range(i1,i2+1):
            for j in range(j1,j2+1):
                region_id = (i,j)
                obstacles,_ = self.world.GetRegion(region_id)
                bounds = self.world.RegionBounds(region_id)
                ibounds = (max(bounds[0]+float_precision,center[0]-unlodge_radius),max(bounds[1]+float_precision,center[1]-unlodge_radius),min(bounds[2]-float_precision,center[0]+unlodge_radius),min(bounds[3]+float_precision,center[1]+unlodge_radius))
                k1,l1 = self.world.TransformPositionToLocalCoordinates((ibounds[0],ibounds[1]))
                k2,l2 = self.world.TransformPositionToLocalCoordinates((ibounds[2],ibounds[3]))
                for k in range(k1,k2+1):
                    row = obstacles[k]
                    for l in range(l1,l2+1):
                        if row[l] > occupancy_lower_threshold:
                            return None


    def RunController(self):
        
        if self.destruction_controller_state == destroyed_state:
            return
        
        global num_calls,average_fps

        _1 = time()

        # if num_calls == 0:
        #     self.__begin_time = time()

        rclpy.spin_once(self.node,timeout_sec=0.01)
        
        self.PlanForGoal()
        
        self.ChaseTarget()

        self.CheckForDestruction()
        
        if abs(self.prev_vel-self.velocity.linear.x) > 2*maximum_linear_speed/linear_smoothing:
            print('anomaly:',self.prev_vel,self.velocity.linear.x)
        self.prev_vel = self.velocity.linear.x

        if self.destruction_controller_state != destroyed_state:
            self.velocity_publisher.publish(self.velocity)
        
        _2 = time()

        # self.VisualizeCurrentRegion()
        # _4 = time()

        # num_calls += 1
        # average_fps += _2-_1
        # print(f'Actual av. fps:{num_calls/(time()-self.__begin_time+float_precision)}')
        # print(f'\rAverage fps:{(num_calls/average_fps)},fps:{1/(_2-_1)}')
        # print('FPS:')
        # print('fps:',1/(_2-_1))
        # print('execute_plan:',1/(_3-_2))
        # print('visualize:',1/(_4-_3))
        # print('overall:',1/(_5-_1))
        # print()

    def DestroyController(self):
        self.destruction_controller_state = 1
    
    def VisualizeCurrentRegion(self):

        if self.position is None:
            return

        state = self.visualizer_controller_state

        if state == visualization_state:

            region_id = self.world.TransformPositionToRegion(self.position)
            complete_region_data = self.world.GetRegion(region_id)
            region_data = complete_region_data[0]
            image_map = np.full((region_data.shape[0],region_data.shape[1],3),128,np.uint8)

            for i in range(image_map.shape[0]):
                for j in range(image_map.shape[1]):
                    if region_data[i,j] < unoccupancy_upper_threshold:
                        image_map[i,j] = [255,255,255]
                    elif region_data[i,j] > occupancy_lower_threshold:
                        image_map[i,j] = [0,0,0]
            
            if self.plan:
                prev_local = None
                for pose in self.plan:
                    if self.world.TransformPositionToRegion(pose) == region_id:
                        i,j = self.world.TransformPositionToLocalCoordinates(pose)
                        if prev_local is not None:
                            image_map = cv2.line(image_map,prev_local,(j,i),(255,0,0),plan_line_thickness)
                        prev_local = (j,i)
                begin = self.world.TransformPositionToLocalCoordinates(self.plan[0])
                end = self.world.TransformPositionToLocalCoordinates(self.plan[-1])
                image_map = cv2.circle(image_map,(begin[1],begin[0]),plan_point_radius,(0,255,255),cv2.FILLED)
                cv2.circle(image_map,(end[1],end[0]),plan_point_radius,(0,255,0),cv2.FILLED)
            
            image_map = np.rot90(image_map,1)
            image_map = np.fliplr(image_map)
            cv2.imshow('Map',image_map)
            region_data = complete_region_data[1]
            image_map = np.zeros((region_data.shape[0],region_data.shape[1],3),np.uint8)
            for i in range(region_data.shape[0]):
                for j in range(region_data.shape[1]):
                    x = region_data[i,j]
                    if x == obstacle_identifier:
                        image_map[i,j] = [0,0,0]
                    elif x == no_visibility_identifier:
                        image_map[i,j] = [0,255,255]
                    elif x == low_visibility_identifier:
                        image_map[i,j] = [255,255,0]
                    elif x == high_visibility_identifier:
                        image_map[i,j] = [255,255,255]
                    elif x == 11:
                        image_map[i,j] = [255,0,255]
            if self.plan:
                prev_local = None
                for pose in self.plan:
                    if self.world.TransformPositionToRegion(pose) == region_id:
                        i,j = self.world.TransformPositionToLocalCoordinates(pose)
                        if prev_local is not None:
                            image_map = cv2.line(image_map,prev_local,(j,i),[255,0,0],plan_line_thickness)
                        prev_local = (j,i)
                begin = self.world.TransformPositionToLocalCoordinates(self.plan[0])
                end = self.world.TransformPositionToLocalCoordinates(self.plan[-1])
                image_map = cv2.circle(image_map,(begin[1],begin[0]),plan_point_radius,[0,255,0],cv2.FILLED)
                cv2.circle(image_map,(end[1],end[0]),plan_point_radius,[0,0,255],cv2.FILLED)
            
            image_map = np.rot90(image_map,1)
            image_map = np.fliplr(image_map)
            cv2.imshow('Planning Map',image_map)
            cv2.waitKey(100)
        
            state = 0

        else: state += 1

        self.visualizer_controller_state = state