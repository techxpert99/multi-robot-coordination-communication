from math import asin,acos,atan2, ceil, cos, exp, floor, log, sin,sqrt,pi, tan
from queue import PriorityQueue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data
from os import makedirs,remove
from os.path import isdir
import numpy as np
from collections import OrderedDict
from sensor_msgs.msg import LaserScan
import cv2
import rclpy

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
    roll,pitch,yaw = TransformQuaternion(msg.pose.pose.orientation.x,msg.pose.pose.orientation.y,msg.pose.pose.orientation.z,msg.pose.pose.orientation.w)
    return (msg.pose.pose.position.x,msg.pose.pose.position.y,yaw)


class RegionCache:
    
    def __init__(self,memory_cache_size=128,cache_size=1024,cache_location='cached_regions/'):
        self.memory_cache_size = memory_cache_size
        self.cache_size = cache_size
        self.cache_location = cache_location
        if not isdir(cache_location):
            makedirs(cache_location)
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
            if len(self.cached_regions) == self.cache_size:
                if self.cache_size > self.memory_cache_size:
                    delete_region_id = self.cached_regions.popitem(False)[0]
                    remove(self.cache_location+'/'+str(delete_region_id[0])+','+str(delete_region_id[1]))
                    evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)[0]
                    if self.cached_regions[evict_region_id]:
                        np.save(self.cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data,allow_pickle=False)
                else:
                    self.cached_regions.popitem(False)
                    self.regions_in_memory.popitem(False)
            elif len(self.regions_in_memory) == self.memory_cache_size:
                evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)[0]
                if self.cached_regions[evict_region_id]:
                    np.save(self.cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data,allow_pickle=False)
        self.cached_regions[region_id] = True
        self.regions_in_memory[region_id] = region_data

    # Gets a region
    def GetRegion(self,region_id):
        if region_id not in self.cached_regions:
            return
        elif region_id in self.regions_in_memory:
            modified = self.cached_regions[region_id]
            region_data = self.regions_in_memory[region_id]
        else:
            if len(self.regions_in_memory) == self.memory_cache_size:
                evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)[0]
                if self.cached_regions[evict_region_id]:
                    np.save(self.cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data,allow_pickle=False)
            region_data = np.load(self.cache_location+'/'+str(region_id[0])+','+str(region_id[1]),None,False)
            modified = False
        del self.regions_in_memory[region_id]
        self.cached_regions[region_id] = modified
        self.regions_in_memory[region_id] = region_data
        return region_data


# Region denotations:
# Obstacles: 0
# Obstacle-free cell: 100
# Unexplored area: 200

class World:

    def __init__(self,region_length,resolution,origin=(0,0)):
        self.origin = origin

        self.region_length = region_length
        self.region_data_length = floor(region_length/resolution)
        
        self.resolution = resolution
        self.cache = RegionCache()

    # Transforms a position (x,y) to a region id (a,b)
    def TransformPositionToRegion(self,position):
        return floor(floor((position[0]-self.origin[0])/self.resolution+0.5)/self.region_data_length),floor(floor((position[1]-self.origin[1])/self.resolution+0.5)/self.region_data_length)
    
    # Transforms a position (x,y) to local coordinates (i,j)
    def TransformPositionToLocalCoordinates(self,position):
        l = floor((position[0]-self.origin[0])/self.resolution+0.5)
        m = floor((position[1]-self.origin[1])/self.resolution+0.5)
        a = floor(l/self.region_data_length)
        b = floor(m/self.region_data_length)
        j = l-a*self.region_data_length
        i = m-b*self.region_data_length
        return i,j
    
    def TransformLocalCoordinatesToPosition(self,region_id,local_coordinates):
        return self.origin[0]+(self.region_data_length*region_id[0]+local_coordinates[1])*self.resolution,self.origin[1]+(self.region_data_length*region_id[1]+local_coordinates[0])*self.resolution

    def GetRegion(self,region_id):
        region_data = None
        # Check if the source region is cached. If it is get it, otherwise create a new region, cache it and get it
        if self.cache.HasRegion(region_id):
            region_data = self.cache.GetRegion(region_id)
        else:
            region_data = np.full((self.region_data_length,self.region_data_length),0,np.float32)
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


    def LocalPlanPath(self,source_position,target_position,allow_unknown=True,obstacle_expansion=3):

        source_region_id = self.TransformPositionToRegion(source_position)
        target_region_id = self.TransformPositionToRegion(target_position)
        
        # If the source position and the target position are in different regions, local planning cannot proceed
        if source_region_id != target_region_id: return None
        
        source_cell = self.TransformPositionToLocalCoordinates(source_position)
        target_cell = self.TransformPositionToLocalCoordinates(target_position)
        region_data = self.GetRegion(source_region_id)

        disallowed_cells = [0]
        if not allow_unknown:
            disallowed_cells.append(1)

        if region_data[target_cell[0]][target_cell[1]] in disallowed_cells: return None

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
                return path
            
            i,j = cell

            for i2,j2 in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i-1,j-1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]:
                child_cell = (i2,j2)
                if i2 >= 0 and j2 >= 0 and i2 < num_rows and j2 < num_cols and child_cell not in explored:
                    if not np.allclose(child_cell,source_cell,atol=2):
                        if region_data[child_cell] in disallowed_cells: continue
                        obstacle = False
                        for i3 in range(i2-obstacle_expansion,i2+obstacle_expansion+1):
                            for j3 in range(j2-obstacle_expansion,j2+obstacle_expansion+1):
                                if region_data[i3,j3] in disallowed_cells:
                                    obstacle = True
                                    break
                            if obstacle:
                                break
                        if obstacle: continue
                    gchild = g + EuclideanDistance(cell,child_cell)
                    if child_cell not in gcosts or gcosts[child_cell] > gchild:
                        fchild = gchild + EuclideanDistance(child_cell,target_cell)
                        frontier.put((fchild,child_cell))
                        gcosts[child_cell] = gchild
                        parents[child_cell] = cell
        
        return None

    def GetAtPosition(self,position):
        region_id = self.TransformPositionToRegion(position)
        local_coordinates = self.TransformPositionToLocalCoordinates(position)
        return self.GetRegion(region_id)[local_coordinates]
    
    def SetAtPosition(self,position,value):
        region_id = self.TransformPositionToRegion(position)
        local_coordinates = self.TransformPositionToLocalCoordinates(position)
        region_data = self.GetRegion(region_id)
        region_data[local_coordinates] = value
        self.SetRegion(region_id,region_data)
    
    def RegionBounds(self,region_id):
        return self.origin[0]+(region_id[0]-0.5)*self.region_length,self.origin[1]+(region_id[1]-0.5)*self.region_length,self.origin[0]+(region_id[0]+0.5)*self.region_length,self.origin[1]+(region_id[1]+0.5)*self.region_length

class Controller:
    
    def __init__(self, parameter_store, robot_namespace, linear_speed, angular_speed, region_length=50, resolution=0.1, sensor_range=9.0, linear_precision=0.1, angular_precision=0.02):
        self.node = Node('controller',namespace=robot_namespace)
        self.parameter_store = parameter_store
        
        self.velocity_publisher = self.node.create_publisher(Twist,robot_namespace+'/velocity',qos_profile_system_default)
        self.odometry_subscription = self.node.create_subscription(Odometry,robot_namespace+'/odometery',self.UpdatePosition,qos_profile_sensor_data)
        self.lidar_subscription = self.node.create_subscription(LaserScan,robot_namespace+'/laser/out',self.UpdateLidarData,qos_profile_sensor_data)
        self.world = World(region_length,resolution)

        self.position = None
        self.plan = None
        self.plan_index = None
        self.target = None
        self.velocity = None
        self.initial_point = None
        self.initial_index = None
        self.target_reached = True

        self.linear = linear_speed
        self.angular = angular_speed
        self.linear_precision = linear_precision
        self.angular_precision = angular_precision

        self.plan_changed = False

        self.sensor_range = sensor_range
        self.resolution = resolution
        self.lidar_spread = 0.1
        self.lidar_max_prob = 0.8
        self.lidar_min_prob = 0.2
        self.lidar_tolerance_prob = 0.2

        self.sensor_time = 0
        self.visualization_time = 0
        self.sensor_time_period = 2
        self.visualization_time_period = 10

        self.CalculateLidarConstants()

        #self.parameter_store.set('controller','position',None)

    def UpdatePosition(self, msg: Odometry):
        self.position = UnmarshalPosition(msg)
        #self.parameter_store.set('controller','position',self.position)
    

    def TransformRobotCoordinatesToPosition(self,radial_distance,angle):
        return self.position[0]+radial_distance*sin(self.position[2]+angle),self.position[1]-radial_distance*cos(self.position[2]+angle)

    def CalculateLidarConstants(self):
        self.lidar_dr1 = sqrt(self.lidar_spread*log(self.lidar_max_prob/self.lidar_min_prob))
        self.lidar_dr2 = sqrt(self.lidar_spread*log(2*self.lidar_max_prob/(self.lidar_max_prob+self.lidar_min_prob)))
        self.lidar_min_prob_log = log(self.lidar_min_prob/(1-self.lidar_min_prob))
        self.lidar_max_prob_log = log(self.lidar_max_prob/(1-self.lidar_max_prob))
        lidar_free_prob_limit = (self.lidar_max_prob+self.lidar_min_prob-self.lidar_tolerance_prob)/2
        lidar_occupied_prob_limit = (self.lidar_max_prob+self.lidar_min_prob+self.lidar_tolerance_prob)/2
        self.lidar_free_prob_log_limit = log(lidar_free_prob_limit/(1-lidar_free_prob_limit))
        self.lidar_occupied_prob_log_limit = log(lidar_occupied_prob_limit/(1-lidar_occupied_prob_limit))
    
    def UpdateLidarData(self, msg: LaserScan):
        
        if self.sensor_time:
            self.sensor_time = (self.sensor_time+1)%self.sensor_time_period
            return
        self.sensor_time = (self.sensor_time+1)%self.sensor_time_period
        

        if not self.position: return
        
        sensor_data = msg.ranges
        dt = msg.angle_increment
        dr = self.resolution

        min_pose = (self.position[0]-self.sensor_range,self.position[1]-self.sensor_range)
        max_pose = (self.position[0]+self.sensor_range,self.position[1]+self.sensor_range)
        
        i1,j1 = self.world.TransformPositionToRegion(min_pose)
        i2,j2 = self.world.TransformPositionToRegion(max_pose)
        
        for i in range(i1,i2+1):
            for j in range(j1,j2+1):
                
                region_id = (i,j)
                region_data = self.world.GetRegion(region_id)
                
                t = 0
                for z in sensor_data:
                    r=0
                    rmax = min(self.sensor_range,z+self.lidar_dr2)
                    while r <= rmax:
                        position = self.TransformRobotCoordinatesToPosition(r,t)
                        if region_id == self.world.TransformPositionToRegion(position):
                            local_position = self.world.TransformPositionToLocalCoordinates(position)
                            if r < z-self.lidar_dr1:
                                region_data[local_position] = max(self.lidar_min_prob_log,region_data[local_position]+self.lidar_min_prob_log)
                            else:
                                p = self.lidar_max_prob*exp(-(r-z)*(r-z)/self.lidar_spread)
                                region_data[local_position] = max(self.lidar_min_prob_log,min(self.lidar_max_prob_log,region_data[local_position]+log(p/(1-p))))
                        r += dr
                    t += dt

                self.world.SetRegion(region_id,region_data)

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
        if self.plan_changed:
            self.plan_changed = False
            self.plan_index = None
            self.initial_point = None
            self.initial_index = None
            self.target_reached = False
        
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
        if not self.plan: return
        if self.plan_index == -1:
            self.target = self.initial_point
            self.plan_index = self.initial_index
        elif self.target_reached:
            if self.plan_index < len(self.plan):
                self.target = self.plan[self.plan_index]
                self.plan_index += 1
                self.target_reached = False
            else:
                self.plan = None
                self.plan_index = None
                self.target = None
                self.initial_index = None
                self.initial_point = None
                self.plan = None

    def ExecutePlan(self):
        self.UpdatePlan()
        self.DecideTarget()
        self.ComputeVelocity()
        if self.velocity is not None:
            self.velocity_publisher.publish(self.velocity)

    def RunController(self):
        rclpy.spin_once(self.node,timeout_sec=0.01)
        self.ExecutePlan()
        self.VisualizeCurrentRegion()

    def DestroyController(self):
        self.node.destroy_node()
    
    def VisualizeCurrentRegion(self):

        if self.visualization_time:
            self.visualization_time = (self.visualization_time+1)%self.visualization_time_period
            return
        self.visualization_time = (self.visualization_time+1)%self.visualization_time_period

        if self.position is None:
            return
        region_id = self.world.TransformPositionToRegion(self.position)
        region_data = self.world.GetRegion(region_id)
        image_map = np.zeros(region_data.shape,np.uint8)
        for i in range(image_map.shape[0]):
            for j in range(image_map.shape[1]):
                if region_data[i,j] < self.lidar_free_prob_log_limit:
                    image_map[i,j] = 255
                elif region_data[i,j] > self.lidar_occupied_prob_log_limit:
                    image_map[i,j] = 0
                else:
                    image_map[i,j] = 128
        cv2.imshow('Map',image_map)
        cv2.waitKey(500)