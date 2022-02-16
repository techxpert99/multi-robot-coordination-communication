from time import time
from controller.wrappers import ThreadWrapper
from math import cos, floor, sin,sqrt,acos
from queue import PriorityQueue


def file_print(*s):
    with open('/home/ghost/Desktop/dev_ws/src/controller/controller/algorithms/save_file.txt','a+') as f:
        for k in range(len(s)-1):
            f.write(str(s[k])+' ')
        if len(s) > 0:
            f.write(str(s[len(s)-1]))
        f.write('\n')

def file_write(*s):
    with open('/home/ghost/Desktop/dev_ws/src/controller/controller/algorithms/save_file.txt','a+') as f:
        for k in range(len(s)-1):
            f.write(str(s[k])+' ')
        if len(s) > 0:
            f.write(str(s[len(s)-1]))


class LocalPlanner:

    def __init__(self,controller_node,store):
        self._store = store
        self._store.set('planner','plan_tf',[])
        self._store.set('planner','plan',[])
        self._weqtime = self._store.get('general','world_inter_regional_cache_updation_period')
        self._leq_update = dict()
        self.__tw__ = ThreadWrapper(self.__callback__,store.get('general','planner_node_callback_interval'))
        self.__tw__.__thread__.setName(store.get('names','namespace')+'/planner')
        self.__tw__.start()
        self.__controller__ = controller_node
        self._analysis_radius = round(store.get('general','planner_path_analysis_radius')/store.get('general','map_resolution'))
    
    def destroy(self):
        self.__tw__.stop()

    # def __neighbour_dist__(self,cell,neighbour):
    #     if cell[0]-neighbour[0] and cell[1]-neighbour[1]: return sqrt(2)
    #     return 1
    
    # def __angle_diff__(self,parents_parent,parent,child):
    #     diff = (parent[0]-parents_parent[0])*(child[0]-parent[0])+(parent[1]-parents_parent[1])*(child[1]-parent[1])
    #     return acos(diff/(self.__neighbour_dist__(parents_parent,parent)*self.__neighbour_dist__(parent,child)))

    # def __movement_cost__(self,parent_cell,child_cell,parent_map,initial_yaw):
    #     cost_per_radian = 10
    #     parents_parent_cell = parent_map[parent_cell]
    #     ang = 0
    #     if parents_parent_cell is not None:
    #         ang = self.__angle_diff__(parents_parent_cell,parent_cell,child_cell)
    #     else:
    #         ang = cos(initial_yaw)*(child_cell[1]-parent_cell[1])+sin(initial_yaw)*(parent_cell[0]-child_cell[0])
    #         ang /= self.__neighbour_dist__(parent_cell,child_cell)
    #         ang = acos(ang)
        
    #     return cost_per_radian*ang + self.__heuristic_cost__(parent_cell,child_cell)
    
    # def __heuristic_cost__(self,start_cell,goal_cell):
    #     cost_per_meter = 1
    #     dx = start_cell[0]-goal_cell[0]
    #     dy = start_cell[1]-goal_cell[1]
    #     return cost_per_meter*sqrt(dx*dx+dy*dy)

    # def __find_path__(self,start_cell,goal_cell,obstacles,initial_yaw):

    #     cost_hyper_critical = 100

    #     rows,cols = obstacles.shape

    #     if start_cell[0] < 0 or start_cell[0] >= rows or start_cell[1] < 0 or start_cell[1] >= cols:
    #         return []
    #     if goal_cell[0] < 0 or goal_cell[0] >= rows or goal_cell[1] < 0 or goal_cell[1] >= cols:
    #         return []
    #     if obstacles[start_cell[0]][start_cell[1]] in [0,1] or obstacles[goal_cell[0]][goal_cell[1]] in [0,1]:
    #         return []

    #     frontier = PriorityQueue()
    #     explored = set()
    #     cost_map = dict()
    #     parent_map = dict()

    #     initial_cost = self.__heuristic_cost__(start_cell,goal_cell)
    #     frontier.put((initial_cost,start_cell))
    #     cost_map[start_cell] = (initial_cost,0)
    #     parent_map[start_cell] = None

    #     while not frontier.empty():
    #         f_cost,(i,j) = frontier.get_nowait()
    #         f_cost,g_cost = cost_map[(i,j)]
    #         explored.add((i,j))

    #         if (i,j) == goal_cell:
    #             cell = goal_cell
    #             path = []
    #             while cell is not None:
    #                 path.append(cell)
    #                 cell = parent_map[cell]
    #             path.reverse()
    #             return path
            
    #         for x,y in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i-1,j-1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]:
    #             if x >= 0 and y >= 0 and x < rows and y < cols and (x,y) not in explored and obstacles[x,y] not in [0,1]:
    #                 g = g_cost + self.__movement_cost__((i,j),(x,y),parent_map,initial_yaw)
    #                 if obstacles[x,y] == 2:
    #                     g += cost_hyper_critical
    #                 if (x,y) not in cost_map or cost_map[(x,y)][1] > g:
    #                     f = self.__heuristic_cost__((x,y),goal_cell) + g
    #                     frontier.put((f,(x,y)))
    #                     cost_map[(x,y)] = (f,g)
    #                     parent_map[(x,y)] = (i,j)
        
    #     return []
    
    # def __reduce_path__(self,path):
    #     if len(path) < 3: return path
    #     reduced_path = [path[0]]
    #     old_direction = (path[1][0]-path[0][0],path[1][1]-path[0][1])
    #     for i in range(2,len(path)):
    #         new_direction = (path[i][0]-path[i-1][0],path[i][1]-path[i-1][1])
    #         if old_direction != new_direction:
    #             reduced_path.append(path[i-1])
    #             old_direction = new_direction
    #     reduced_path.append(path[-1])
    #     return reduced_path
    
    # def _is_cell_blocked(self,i,j,t,map):
    #     for k in range(i-t,i+t+1):
    #         for l in range(j-t,j+t+1):
    #             if k >= 0 and l >= 0 and k < map.shape[0] and l < map.shape[1] and map[k,l] in [0,1]:
    #                 return True
    #     return False

    # def _is_segment_blocked(self,i1,j1,i2,j2,t,map):
    #     if i1 != i2:
    #         if i1>i2:
    #             i1,i2 = i2,i1
    #             j1,j2 = j2,j1
    #         m = (j2-j1)/(i2-i1)
    #         for i in range(i1,i2+1):
    #             j = floor(m*(i-i1)+j1)
    #             if self._is_cell_blocked(i,j,t,map):
    #                 return True

    #     elif j1 != j2:
    #         if j1>j2:
    #             j1,j2 = j2,j1
    #         for j in range(j1,j2+1):
    #             if self._is_cell_blocked(i1,j,t,map):
    #                 return True
    #     else:
    #         if self._is_cell_blocked(i1,j1,t,map):
    #             return True
        
    #     return False
    
    # def _is_path_blocked(self,path,map,t):
    #     for k in range(1,len(path)):
    #         if self._is_segment_blocked(path[k-1][0],path[k-1][1],path[k][0],path[k][1],t,map):
    #             return True
    #     return False
    
    def __callback__(self):

        world = self._store.get('mapper','world')
        
        if self._store.has('estimator','position'):
            self._store.lock('mapper')
            rid = world.current_region_id()
            if rid not in self._leq_update or time()-self._leq_update[rid] >= self._weqtime:
                print('inter_regional_cache_update_interrupt')
                self._store.set('critical','interrupt',True)
                self.__controller__.critical_stop()
                world.cache_inter_regional_paths()
                self._store.release('mapper')
                self._store.set('critical','interrupt',False)
                self._leq_update[rid] = time()
            else:
                self._store.release('mapper')

        if not self._store.has('planner','goal') or self._store.get('planner','goal') is None:
            return

        goal_point = self._store.get('planner','goal')
        position = self._store.get('estimator','position')
        start_point = position[0],position[1]
        plan = self._store.get('planner','plan')
        self._store.lock('mapper')
        # region = world.get_region(start_point)

        if plan is not None and len(plan) > 1 and world.verify_global_plan(plan):
            self._store.release('mapper')
            return
        
        self._store.release('mapper')

        print('plan_change_interrupt')
        self._store.set('critical','interrupt',True)
        self.__controller__.critical_stop()
        self._store.lock('mapper')
        plan = world.global_plan(start_point,goal_point,position[2])
        reduced_plan = world.reduce_global_plan(plan)
        plan_tf = world.transform_global_plan(reduced_plan)
        self._store.set('planner','plan',plan)
        self._store.set('planner','plan_tf',plan_tf)
        # file_print('Start Point:',start_point)
        # file_print('Goal Point:',goal_point)
        # file_print('Deduced Plan:',plan)
        # file_print('Reduced Plan:',reduced_plan)
        # file_print()
        self._store.release('mapper')
        self._store.set('critical','interrupt',False)

        # return

        # world = self._store.get('mapper','world')
        # _cmap = cmap.get_critical_map()

        # self._store.lock('mapper')
        # if _cmap is None:
        #     self._store.release('mapper')
        #     return
        # self._store.release('mapper')
        
        # if not self._store.has('planner','goal') or self._store.get('planner','goal') is None:
        #     x,y,yaw = self._store.get('estimator','position')
        #     from controller.comapper import Region
        #     self._store.lock('mapper')
        #     reg = Region(_cmap,0.5,9.0)
        #     goal = reg.find_unexplored_goal(cmap.itf(x,y),yaw)
        #     if goal is None: return
        #     goal = cmap.tf(goal[0],goal[1])
        #     self._store.release('mapper')
        #     self._store.lock('planner')
        #     self._store.unsafe_set('planner','goal',goal)
        #     self._store.unsafe_set('planner','plan',[])
        #     self._store.release('planner')
        #     print('New Goal:',goal)
        #     return

        # goal_point = self._store.get('planner','goal')
        # start_point = self._store.get('estimator','position')
        # plan_tf = self._store.get('planner','plan')

        # self._store.lock('mapper')
        # if plan_tf and len(plan_tf) >= 2:
        #     plan = []
        #     for x,y in plan_tf:
        #         plan.append(cmap.itf(x,y))
        #     if not self._is_path_blocked(plan,_cmap,self._analysis_radius):
        #         self._store.release('mapper')
        #         return
        # self._store.release('mapper')
        
        # self._store.set('critical','interrupt',True)
        # self.__controller__.critical_stop()
        
        # self._store.lock('mapper')
        # start = cmap.itf(start_point[0],start_point[1])
        # goal = cmap.itf(goal_point[0],goal_point[1])
        # plan = self.__reduce_path__( self.__find_path__(start,goal,_cmap,start_point[2]))
        # plan_tf = []
        # for i,j in plan:
        #     plan_tf.append(cmap.tf(i,j))
        # self._store.release('mapper')

        # if len(plan_tf) < 2:
        #     self._store.set('planner','goal',None)

        # self._store.set('planner','plan',plan_tf)
        # self._store.set('critical','interrupt',False)