from math import asin, acos, atan2, cos, exp, floor, log, sin, sqrt, pi
from queue import PriorityQueue
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
    
    def __init__(self,memory_cache_size=16,cache_size=128,cache_location='cached_regions/'):
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
                    remove(self.cache_location+'/'+str(delete_region_id[0])+','+str(delete_region_id[1])+'.npy')
                    evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)
                    if self.cached_regions[evict_region_id]:
                        np.save(self.cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data,allow_pickle=False)
                else:
                    self.cached_regions.popitem(False)
                    self.regions_in_memory.popitem(False)
            elif len(self.regions_in_memory) == self.memory_cache_size:
                evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)
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
            del self.regions_in_memory[region_id]
        else:
            if len(self.regions_in_memory) == self.memory_cache_size:
                evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)
                if self.cached_regions[evict_region_id]:
                    np.save(self.cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data,allow_pickle=False)
            region_data = np.load(self.cache_location+'/'+str(region_id[0])+','+str(region_id[1])+'.npy',None,False)
            modified = False
        del self.cached_regions[region_id]
        self.cached_regions[region_id] = modified
        self.regions_in_memory[region_id] = region_data
        return region_data

    # Loads the cache with pre-saved information
    def InitialLoad(self):
        if not isfile(self.cache_location+'/meta.npy'):
            return
        region_ids = np.load(self.cache_location+'/meta.npy',None,False)
        num_loads = min(self.cache_size-len(self.regions_in_memory),len(region_ids))
        for i in range(num_loads):
            region_id = tuple(region_ids[num_loads-1-i])
            self.regions_in_memory[region_id] = np.load(self.cache_location+'/'+str(region_id[0])+','+str(region_id[1])+'.npy',None,False)
        for i in range(len(region_ids)):
            self.cached_regions[tuple(region_ids[len(region_ids)-1-i])] = False
    
    # Flushes the contents of the cache to the disk
    def Flush(self):
        for region_id in self.regions_in_memory:
            if self.cached_regions[region_id]:
                region_data = self.regions_in_memory[region_id]
                np.save(self.cache_location+'/'+str(region_id[0])+','+str(region_id[1]),region_data,allow_pickle=False)
        region_ids = np.zeros((len(self.cached_regions),2),np.int32)
        i = 0
        for region_id in self.cached_regions:
            region_ids[i] = region_id
            i += 1
        np.save(self.cache_location+'/meta',region_ids,allow_pickle=False)
        

# Region denotations:
# Obstacles: 0
# Obstacle-free cell: 100
# Unexplored area: 200

class World:

    def __init__(self):

        #Constants
        self.origin = (0,0)
        self.region_length = 50
        self.resolution = 0.1
        self.region_data_length = floor(self.region_length/self.resolution)

        #Region Cache        
        self.cache = RegionCache()
        self.cache.InitialLoad()
        print('Loading cache')

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


    def LocalPlanPath(self,source_position,target_position):
        
        # Constants
        max_prob = 0.8
        min_prob = 0.2
        prob_tolerance = 0.2
        occupied_prob_limit = (max_prob+min_prob+prob_tolerance)/2
        occupied_prob_log_limit = log(occupied_prob_limit/(1-occupied_prob_limit))
        obstacle_expansion=3

        source_region_id = self.TransformPositionToRegion(source_position)
        target_region_id = self.TransformPositionToRegion(target_position)
        
        # If the source position and the target position are in different regions, local planning cannot proceed
        if source_region_id != target_region_id: return None
        
        source_cell = self.TransformPositionToLocalCoordinates(source_position)
        target_cell = self.TransformPositionToLocalCoordinates(target_position)
        region_data = self.GetRegion(source_region_id)

        if region_data[target_cell[0]][target_cell[1]] > occupied_prob_log_limit: return None

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

            for i2,j2 in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i-1,j-1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]:
                child_cell = (i2,j2)
                if i2 >= 0 and j2 >= 0 and i2 < num_rows and j2 < num_cols and child_cell not in explored:
                    if not np.allclose(child_cell,source_cell,atol=2):
                        if region_data[child_cell] > occupied_prob_log_limit: continue
                        obstacle = False
                        for i3 in range(i2-obstacle_expansion,i2+obstacle_expansion+1):
                            for j3 in range(j2-obstacle_expansion,j2+obstacle_expansion+1):
                                if region_data[i3,j3] > occupied_prob_log_limit:
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
    
    def __init__(self, parameter_store, robot_namespace):
        self.node = Node('controller',namespace=robot_namespace)
        self.parameter_store = parameter_store
        
        self.velocity_publisher = self.node.create_publisher(Twist,robot_namespace+'/velocity',qos_profile_system_default)
        self.odometry_subscription = self.node.create_subscription(Odometry,robot_namespace+'/odometery',self.UpdatePosition,qos_profile_sensor_data)
        self.lidar_subscription = self.node.create_subscription(LaserScan,robot_namespace+'/laser/out',self.UpdateMapWithLidarData,qos_profile_sensor_data)
        self.world = World()

        self.position = None
        self.plan = None
        self.target = None
        self.velocity = Twist()

        self.goal_changed = False
        self.goal = None

        self.sensor_time = 0
        self.visualization_time = 0
        self.cache_flush_time = 0
        self.replanning_time = 0

        self.interrupted = None
    
    def IsInterrupted(self): return self.interrupted is not None

    def HandleInterrupt(self):
        
        if self.interrupted is None: return
        
        # Constants
        maximum_linear_velocity = 1.5
        linear_velocity_smoothing=80
        float_precision=0.0001
        max_interrupt_id_plus_1 = 256
        
        # Stop the robot before processing the interrupt
        if abs(self.velocity.angular.z) >= float_precision or self.velocity.linear.x >= float_precision:
            self.velocity.angular.z = 0.
            self.velocity.linear.x = max(0.,self.velocity.linear.x-maximum_linear_velocity/linear_velocity_smoothing)
            self.interrupted -= max_interrupt_id_plus_1
        elif self.interrupted < 0:
            self.interrupted += max_interrupt_id_plus_1
    
    def Interrupt(self,interrupt_id = 0):
        self.interrupted = interrupt_id
    
    def Uninterrupt(self):
        self.interrupted = None

    def UpdatePosition(self, msg: Odometry):
        self.position = UnmarshalPosition(msg)

    def TransformRobotCoordinatesToPosition(self,radial_distance,angle):
        return self.position[0]+radial_distance*sin(self.position[2]+angle),self.position[1]-radial_distance*cos(self.position[2]+angle)

    def UpdateMapWithLidarData(self, msg: LaserScan):

        #Constants
        spread = 0.1
        max_prob = 0.8
        min_prob = 0.2
        sensor_range = 9.0
        resolution = 0.1
        dr1 = sqrt(spread*log(max_prob/min_prob))
        dr2 = sqrt(spread*log(2*max_prob/(max_prob+min_prob)))
        min_prob_log = log(min_prob/(1-min_prob))
        max_prob_log = log(max_prob/(1-max_prob))
        callback_period = 10

        self.sensor_time = (self.sensor_time+1)%callback_period
        if self.sensor_time: return

        if not self.position: return
        sensor_data = msg.ranges
        dt = msg.angle_increment
        dr = resolution
        min_pose = (self.position[0]-sensor_range,self.position[1]-sensor_range)
        max_pose = (self.position[0]+sensor_range,self.position[1]+sensor_range)
        i1,j1 = self.world.TransformPositionToRegion(min_pose)
        i2,j2 = self.world.TransformPositionToRegion(max_pose)
        for i in range(i1,i2+1):
            for j in range(j1,j2+1):
                region_id = (i,j)
                region_data = self.world.GetRegion(region_id)
                t = 0
                for z in sensor_data:
                    r=0
                    rmax = min(sensor_range,z+dr2)
                    while r <= rmax:
                        position = self.TransformRobotCoordinatesToPosition(r,t)
                        if region_id == self.world.TransformPositionToRegion(position):
                            local_position = self.world.TransformPositionToLocalCoordinates(position)
                            if r < z-dr1:
                                region_data[local_position] = max(min_prob_log,region_data[local_position]+min_prob_log)
                            else:
                                p = max_prob*exp(-(r-z)*(r-z)/spread)
                                region_data[local_position] = max(min_prob_log,min(max_prob_log,region_data[local_position]+log(p/(1-p))))
                        r += dr
                    t += dt
                self.world.SetRegion(region_id,region_data)

    def ComputeVelocity(self):
        
        if self.IsInterrupted(): return

        # Constants
        linear_tolerance=0.5
        angular_tolerance=0.05
        reorientation_tolerance=0.2
        linear_velocity_smoothing=80
        angular_velocity_smoothing=1
        maximum_linear_velocity = 1.5
        maximum_angular_velocity = 0.75
        float_precision=0.0001

        if not self.position or not self.target:
            self.velocity.linear.x = 0.
            self.velocity.angular.z = 0.
            return
        current_source = self.position
        current_target = self.target[0],self.target[1],self.target[2]
        new_target = self.target[3]
        self.target = self.target[0],self.target[1],self.target[2],False
        current_linear_velocity = self.velocity.linear.x
        current_angular_velocity = self.velocity.angular.z
        current_distance_from_target = EuclideanDistance(current_source, current_target)
        target_reached = True
        next_linear_velocity = 0.
        next_angular_velocity = 0.
        if current_distance_from_target >= linear_tolerance:
            current_displacement_from_target = [current_target[0]-current_source[0],current_target[1]-current_source[1]]
            current_angle_from_target = acos(current_displacement_from_target[0]/current_distance_from_target)
            if current_displacement_from_target[1]<0: current_angle_from_target *= -1.
            current_angle_difference = AngleDifference(current_angle_from_target,current_source[2])
            if (new_target and abs(current_angle_difference) >= angular_tolerance) or (not new_target and abs(current_angle_difference) >= reorientation_tolerance):
                next_linear_velocity = max(0.,current_linear_velocity-maximum_linear_velocity/linear_velocity_smoothing)
                if current_linear_velocity < float_precision:
                    if current_angle_difference > 0:
                        next_angular_velocity = max(current_angular_velocity-maximum_angular_velocity/angular_velocity_smoothing,-maximum_angular_velocity)
                    else:
                        next_angular_velocity = min(current_angular_velocity+maximum_angular_velocity/angular_velocity_smoothing,maximum_angular_velocity)
            else:
                if current_angular_velocity < 0:
                    next_angular_velocity = min(current_angular_velocity+maximum_angular_velocity/angular_velocity_smoothing,0.)
                else:
                    next_angular_velocity = max(current_angular_velocity-maximum_angular_velocity/angular_velocity_smoothing,0.)
                if abs(current_angular_velocity) < float_precision:
                    next_linear_velocity = min(maximum_linear_velocity,current_linear_velocity+maximum_linear_velocity/linear_velocity_smoothing)
            target_reached = False
        else:
            next_linear_velocity = max(0.,current_linear_velocity-maximum_linear_velocity/linear_velocity_smoothing)
            if current_linear_velocity >= float_precision:
                target_reached = False
        self.velocity.linear.x = next_linear_velocity
        self.velocity.angular.z = next_angular_velocity
        if target_reached: self.target = None
    
    def UpdatePlan(self):

        if self.interrupted not in [None,1]: return

        #Constants
        replanning_period = 50000

        self.replanning_time = (self.replanning_time+1)%replanning_period

        if not self.position or not self.goal or (not self.goal_changed and self.replanning_time): return

        if not self.replanning_time:
            if not self.IsInterrupted():
                self.Interrupt(1)
                self.replanning_time = -1
            else:
                self.target = None
                self.Uninterrupt()

        self.goal_changed = False
        self.target_reached = False
        plan = self.world.PlanPath(self.position,self.goal)
        if not plan: return
        px,py,_ = self.position
        cache = None
        for i in range(1,len(plan)):
            ax,ay = plan[i-1]
            bx,by = plan[i]
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
        plan = plan[cache[2]:]
        plan.insert(0,cache[1])
        self.plan = plan
    
    def DecideTarget(self):
        #Constants
        num_skip_poses = 10

        if not self.plan: return
        if not self.target:
            skip_poses = num_skip_poses
            while skip_poses and self.plan:
                pose = self.plan.pop(0)
                skip_poses -= 1
            if self.plan:
                pose = self.plan.pop(0)
            self.target = pose[0],pose[1],None,True

    def ExecutePlan(self):
        self.UpdatePlan()
        self.DecideTarget()
        self.ComputeVelocity()
        self.velocity_publisher.publish(self.velocity)

    def FlushCache(self):
        #Constants
        cache_flush_time_period = 1200000

        self.cache_flush_time = (self.cache_flush_time+1)%cache_flush_time_period
        if self.cache_flush_time: return

        self.world.cache.Flush()
        print('Flushing cache')

    def RunController(self):
        rclpy.spin_once(self.node,timeout_sec=0.01)        
        self.ExecutePlan()
        self.VisualizeCurrentRegion()

    def DestroyController(self):
        self.node.destroy_node()
        self.world.cache.Flush()
        print('Flushing cache')
    
    def VisualizeCurrentRegion(self):
    
        if self.IsInterrupted(): return

        #Constants
        max_prob = 0.8
        min_prob = 0.2
        prob_tolerance = 0.2
        free_prob_limit = (max_prob+min_prob-prob_tolerance)/2
        occupied_prob_limit = (max_prob+min_prob+prob_tolerance)/2
        free_prob_log_limit = log(free_prob_limit/(1-free_prob_limit))
        occupied_prob_log_limit = log(occupied_prob_limit/(1-occupied_prob_limit))
        callback_period = 1000
        plan_line_thickness = 3
        plan_point_radius = 3

        self.visualization_time = (self.visualization_time+1)%callback_period
        if self.visualization_time: return

        if self.position is None:
            return
        region_id = self.world.TransformPositionToRegion(self.position)
        region_data = self.world.GetRegion(region_id)
        image_map = np.full((region_data.shape[0],region_data.shape[1],3),128,np.uint8)
        region_data < free_prob_log_limit
        for i in range(image_map.shape[0]):
            for j in range(image_map.shape[1]):
                if region_data[i,j] < free_prob_log_limit:
                    image_map[i,j] = [255,255,255]
                elif region_data[i,j] > occupied_prob_log_limit:
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
        cv2.waitKey(100)