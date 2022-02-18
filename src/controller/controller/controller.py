from math import asin,acos,atan2, ceil, floor,sqrt,pi
from queue import PriorityQueue
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from rclpy.qos import qos_profile_system_default
from rclpy.qos import qos_profile_sensor_data
from os import makedirs,remove
import numpy as np
from collections import OrderedDict
from threading import Lock

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


class ColdRegionCache:
    
    def __init__(self,memory_cache_size=128,cache_size=1024,cache_location='cached_regions/'):
        self.memory_cache_size = memory_cache_size
        self.cache_size = cache_size
        self.cache_location = cache_location
        makedirs(self.cache_location)
        self.cached_regions = OrderedDict()
        self.regions_in_memory = OrderedDict()

    # Determines if a region is cached
    def HasRegion(self,region_id):
        return region_id in self.cached_regions
    
    # Inserts a region into the cache
    def InsertRegion(self,region_id,region_data):
        if region_id in self.regions_in_memory:
            del self.cached_regions[region_id]
            del self.regions_in_memory[region_id]
            self.cached_regions[region_id] = None
            self.regions_in_memory[region_id] = region_data
        elif region_id in self.cached_regions:
            if len(self.regions_in_memory) == self.memory_cache_size:
                evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)[0]
                np.save(self.cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data,allow_pickle=False)
            del self.cached_regions[region_id]
            self.cached_regions[region_id] = None
            self.regions_in_memory[region_id] = region_data
        else:
            if len(self.cached_regions) == self.cache_size:
                if self.cache_size > self.memory_cache_size:
                    delete_region_id = self.cached_regions.popitem(False)[0]
                    remove(self.cache_location+'/'+str(delete_region_id[0])+','+str(delete_region_id[1]))
                    evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)[0]
                    np.save(self.cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data,allow_pickle=False)
                else:
                    self.cached_regions.popitem(False)
                    self.regions_in_memory.popitem(False)
            elif len(self.regions_in_memory) == self.memory_cache_size:
                evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)[0]
                np.save(self.cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data,allow_pickle=False)
            self.cached_regions[region_id] = None
            self.regions_in_memory[region_id] = region_data
    
    # Gets a region if it is cached
    def GetRegion(self,region_id):
        region_data = None
        if region_id in self.regions_in_memory:
            region_data = self.regions_in_memory[region_id]
            del self.cached_regions[region_id]
            del self.regions_in_memory[region_id]
            self.cached_regions[region_id] = None
            self.regions_in_memory[region_id] = region_data
        elif region_id in self.cached_regions:
            if len(self.regions_in_memory) == self.memory_cache_size:
                evict_region_id,evict_region_data = self.regions_in_memory.popitem(False)[0]
                np.save(self.cache_location+'/'+str(evict_region_id[0])+','+str(evict_region_id[1]),evict_region_data,allow_pickle=False)
            region_data = np.load(self.cache_location+'/'+str(region_id[0])+','+str(region_id[1]),None,False)
            del self.cached_regions[region_id]
            self.cached_regions[region_id] = None
            self.regions_in_memory[region_id] = region_data
        return region_data

class HotRegionCache:

    def __init__(self,cold_cache: ColdRegionCache, cache_size = 9):
        self.cold_cache = cold_cache
        self.cache_size = cache_size
        self.hot_cache = OrderedDict()
        self.lock = Lock()
    
    def HasRegion(self,region_id):
        self.lock.acquire()
        

# Region denotations:
# Obstacles: 0
# Obstacle-free cell: 100
# Unexplored area: 200

class World:

    def __init__(self,region_length=50,resolution=0.1,origin=(0,0)):
        self.origin = origin

        self.region_length = region_length
        self.region_data_length = floor(region_length/resolution)
        
        self.resolution = resolution
        self.cache = ColdRegionCache()

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
            region_data = np.full((self.region_length,self.region_length),200,np.uint8)
            self.cache.InsertRegion(region_id,region_data)
        return region_data

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


class Controller:
    
    def __init__(self, parameter_store, robot_namespace, linear_speed, angular_speed, linear_precision=0.1, angular_precision=0.02):
        self.node = Node(robot_namespace+'/controller')
        self.parameter_store = parameter_store
        
        self.velocity_publisher = self.node.create_publisher(Twist,robot_namespace+'/velocity',qos_profile_system_default)
        self.odometry_subscription = self.node.create_subscription(Odometry,robot_namespace+'/odometry',self.UpdatePosition,qos_profile_sensor_data)
        
        self.world = World()

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

        self.plan_changed = False

        self.parameter_store.set('controller','position',None)

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
        if self.plan_changed:
            self.plan_changed = False
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
                self.plan = None

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