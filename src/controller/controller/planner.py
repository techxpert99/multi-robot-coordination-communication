from controller.wrappers import ThreadWrapper
from math import sqrt,acos
from queue import PriorityQueue


class LocalPlanner:

    def __init__(self,critical_node,store,plan_interval=0.1):
        self._store = store
        self._store.set('planner','plan',([],[]))
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
        plan_tf = self._store.get('planner','plan')[1]
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
        if self._store.get('mapper','cmap').get_critical_map() is None or not self._store.has('general','goal'): return
        goal_point = self._store.get('general','goal')
        start_point = self._store.get('estimator','position')
        self._store.lock('mapper')
        cmap = self._store.unsafe_get('mapper','cmap')
        # analysis = self.__analyze_plan__(self._store.get('planner','plan')[1],cmap)
        # if analysis == 'UNBLOCKED': return
        # elif analysis == 'BLOCKED':
        #     self.__cc__.__critical_action__()
        start = cmap.itf(start_point[0],start_point[1])
        goal = cmap.itf(goal_point[0],goal_point[1])
        path = self.__reduce_path__( self.__find_path__(start,goal,cmap.get_critical_map()))
        tf_path = []
        for i,j in path:
            tf_path.append(cmap.tf(i,j))
        self._store.release('mapper')
        self._store.set('planner','plan',(path,tf_path))
      
