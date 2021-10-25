from controller.wrappers import ThreadWrapper
from math import cos, floor, sin,sqrt,acos
from queue import PriorityQueue


class LocalPlanner:

    def __init__(self,controller_node,store):
        self._store = store
        self._store.set('planner','plan',([],[]))
        self.__tw__ = ThreadWrapper(self.__callback__,store.get('general','planner_node_callback_interval'))
        self.__tw__.__thread__.setName("planner-thread")
        self.__tw__.start()
        self.__controller__ = controller_node
        self._analysis_radius = round(store.get('general','planner_path_analysis_radius')/store.get('general','map_resolution'))
    
    def destroy(self):
        self.__tw__.stop()

    def __neighbour_dist__(self,cell,neighbour):
        if cell[0]-neighbour[0] and cell[1]-neighbour[1]: return sqrt(2)
        return 1
    
    def __angle_diff__(self,parents_parent,parent,child):
        diff = (parent[0]-parents_parent[0])*(child[0]-parent[0])+(parent[1]-parents_parent[1])*(child[1]-parent[1])
        return acos(diff/(self.__neighbour_dist__(parents_parent,parent)*self.__neighbour_dist__(parent,child)))

    def __movement_cost__(self,parent_cell,child_cell,parent_map,initial_yaw):
        cost_per_radian = 10
        parents_parent_cell = parent_map[parent_cell]
        ang = 0
        if parents_parent_cell is not None:
            ang = self.__angle_diff__(parents_parent_cell,parent_cell,child_cell)
        else:
            ang = cos(initial_yaw)*(child_cell[1]-parent_cell[1])+sin(initial_yaw)*(parent_cell[0]-child_cell[0])
            ang /= self.__neighbour_dist__(parent_cell,child_cell)
            ang = acos(ang)
        
        return cost_per_radian*ang + self.__heuristic_cost__(parent_cell,child_cell)
    
    def __heuristic_cost__(self,start_cell,goal_cell):
        cost_per_meter = 1
        dx = start_cell[0]-goal_cell[0]
        dy = start_cell[1]-goal_cell[1]
        return cost_per_meter*sqrt(dx*dx+dy*dy)

    def __find_path__(self,start_cell,goal_cell,obstacles,initial_yaw):

        cost_hyper_critical = 100

        rows,cols = obstacles.shape

        if start_cell[0] < 0 or start_cell[0] >= rows or start_cell[1] < 0 or start_cell[1] >= cols:
            return []
        if goal_cell[0] < 0 or goal_cell[0] >= rows or goal_cell[1] < 0 or goal_cell[1] >= cols:
            return []
        if obstacles[start_cell[0]][start_cell[1]] in [0,1] or obstacles[goal_cell[0]][goal_cell[1]] in [0,1]:
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
                if x >= 0 and y >= 0 and x < rows and y < cols and (x,y) not in explored and obstacles[x,y] not in [0,1]:
                    g = g_cost + self.__movement_cost__((i,j),(x,y),parent_map,initial_yaw)
                    if obstacles[x,y] == 2:
                        g += cost_hyper_critical
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
    
    def _is_cell_blocked(self,i,j,t,map):
        for k in range(i-t,i+t+1):
            for l in range(j-t,j+t+1):
                if k >= 0 and l >= 0 and k < map.shape[0] and l < map.shape[1] and map[k,l] in [0,1]:
                    return True
        return False

    def _is_segment_blocked(self,i1,j1,i2,j2,t,map):
        
        i1,i2 = min(i1,i2),max(i1,i2)
        j1,j2 = min(j1,j2),max(j1,j2)

        if i1 != i2:
            m = (j2-j1)/(i2-i1)
            for i in range(i1,i2+1):
                j = floor(m*(i-i1)+j1)
                if self._is_cell_blocked(i,j,t,map):
                    return True

        elif j1 != j2:
            for j in range(j1,j2+1):
                if self._is_cell_blocked(i1,j,t,map):
                    return True
        else:
            if self._is_cell_blocked(i1,j1,t,map):
                return True
        
        return False
    
    def _is_path_blocked(self,path,map,t):
        for k in range(1,len(path)):
            if self._is_segment_blocked(path[k-1][0],path[k-1][1],path[k][0],path[k][1],t,map):
                return True
        return False
    
    def __callback__(self):
        if self._store.get('mapper','cmap').get_critical_map() is None or not self._store.has('planner','goal'): return
        
        goal_point = self._store.get('planner','goal')
        start_point = self._store.get('estimator','position')
        plan,plan_tf = self._store.get('planner','plan')

        self._store.lock('mapper')
        cmap = self._store.unsafe_get('mapper','cmap')        
        
        interrupted = False
        if plan and len(plan) >= 2:
            path = []
            for x,y in plan_tf:
                path.append(cmap.itf(x,y))
            if not self._is_path_blocked(path,cmap.get_critical_map(),self._analysis_radius):
                self._store.release('mapper')
                self._store.set('planner','plan',(path,plan_tf))
                return
            self._store.release('mapper')
            self._store.set('critical','interrupt',True)
            self.__controller__.critical_stop()
            interrupted = True
            print('plan_change_interrupt')
            self._store.lock('mapper')
        
        start = cmap.itf(start_point[0],start_point[1])
        goal = cmap.itf(goal_point[0],goal_point[1])
        path = self.__reduce_path__( self.__find_path__(start,goal,cmap.get_critical_map(),start_point[2]))
        tf_path = []
        for i,j in path:
            tf_path.append(cmap.tf(i,j))
        self._store.release('mapper')
        self._store.set('planner','plan',(path,tf_path))
        if interrupted:
            self._store.set('critical','interrupt',False)