from math import acos, ceil, cos, sin, sqrt
from queue import PriorityQueue, Queue
import numpy as np

class LinkedGrid:
    def __init__(self):
        pass

class Region:
    
    def __init__(self,critical_map,resolution,sensor_radius):
        self._cmap = critical_map
        self._costmap = {'hypercritical':100,'radian':10,'meter':1}
        self._infinity = 2**31
        self._radius = sensor_radius
        self._resolution = resolution

    def _neighbour_dist(self,cell,neighbour):
        if cell[0]-neighbour[0] and cell[1]-neighbour[1]: return sqrt(2)
        return 1
    
    def _angle_diff(self,parents_parent,parent,child):
        diff = (parent[0]-parents_parent[0])*(child[0]-parent[0])+(parent[1]-parents_parent[1])*(child[1]-parent[1])
        return acos(diff/(self._neighbour_dist(parents_parent,parent)*self._neighbour_dist(parent,child)))

    def _movement_cost(self,parent_cell,child_cell,parent_map,initial_yaw):
        parents_parent_cell = parent_map[parent_cell]
        ang = 0
        if parents_parent_cell is not None:
            ang = self._angle_diff(parents_parent_cell,parent_cell,child_cell)
        else:
            ang = cos(initial_yaw)*(child_cell[1]-parent_cell[1])+sin(initial_yaw)*(parent_cell[0]-child_cell[0])
            ang /= self._neighbour_dist(parent_cell,child_cell)
            ang = acos(ang)
        return self._costmap['radian']*ang + self._costmap['meter']*self._neighbour_dist(parent_cell,child_cell)

    def _strip_average(self,X,r,INFINITY):
        if X is None or len(X) == 0: return X
        A = list()
        c = Queue()
        n = 0
        s = 0
        for i in range(r+1):
            c.put(INFINITY)
        for i in range(min(r,len(X))):
            c.put(X[i])
            if X[i] != INFINITY:
                n += 1
                s += X[i]
        for i in range(len(X)):
            x = c.get()
            if x != INFINITY:
                n -= 1
                s -= x
            if i+r < len(X):
                c.put(X[i+r])
                if X[i+r] != INFINITY:
                    n += 1
                    s += X[i+r]
            A.append((n,s))
        return A

    def _block_average(self,X,r,INFINITY):
        rows,cols = X.shape
        S = [self._strip_average(X[i],r,INFINITY) for i in range(rows)]
        A = [[INFINITY for j in range(cols)] for i in range(rows)]
        for j in range(cols):
            C = Queue()
            n = 0
            s = 0
            for i in range(r+1):
                C.put((0,0))
            for i in range(min(r,rows)):
                C.put(S[i][j])
                if S[i][j][0] != 0:
                    n += S[i][j][0]
                    s += S[i][j][1]
            for i in range(rows):
                x = C.get()
                if x[0] != 0:
                    n -= x[0]
                    s -= x[1]
                if i+r < rows:
                    C.put(S[i+r][j])
                    if S[i+r][j][0] != 0:
                        n += S[i+r][j][0]
                        s += S[i+r][j][1]
                A[i][j] = (n,s)
        return A

    def find_unexplored_subregion(self,start_cell,initial_yaw):

        rows,cols = self._cmap.shape
        if start_cell[0] < 0 or start_cell[0] >= rows or start_cell[1] < 0 or start_cell[1] >= cols or self._cmap[start_cell] in [0,1]:
            return [[]]

        frontier = PriorityQueue()
        explored = set()
        cost_map = dict()
        parent_map = dict()

        frontier.put((0,start_cell))
        cost_map[start_cell] = 0
        parent_map[start_cell] = None

        unexplored_reachable_region = np.full(shape=self._cmap.shape,fill_value=self._infinity,dtype=np.float64)

        while not frontier.empty():
            cost,(i,j) = frontier.get()
            explored.add((i,j))
            if self._cmap[i,j] == 4:
                unexplored_reachable_region[i,j] = cost
            for x,y in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i-1,j-1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]:
                if x >= 0 and y >= 0 and x < rows and y < cols and (x,y) not in explored and self._cmap[x,y] not in [0,1]:
                    _cost = cost + self._movement_cost((i,j),(x,y),parent_map,initial_yaw)
                    if self._cmap[x,y] == 2:
                        _cost += self._costmap['hypercritical']
                    if (x,y) not in cost_map or cost_map[(x,y)] > _cost:
                        frontier.put((_cost,(x,y)))
                        cost_map[(x,y)] = _cost
                        parent_map[(x,y)] = (i,j)
        
        unexplored_costmap = self._block_average(unexplored_reachable_region,ceil(self._radius/self._resolution),self._infinity)
        x = open('saved_cmaps/hello.txt','w')
        x.write('Reachability Map:\n\n')
        for i in range(unexplored_reachable_region.shape[0]):
            for j in range(unexplored_reachable_region.shape[1]):
                x.write(f'{unexplored_reachable_region[i,j]} ')
            x.write('\n')
        x.write('\n\n\n\nCostmap:\n\n')
        for i in range(len(unexplored_costmap)):
            for j in range(len(unexplored_costmap[0])):
                x.write(f'{unexplored_costmap[i][j]} ')
            x.write('\n')
        x.close()
        return unexplored_costmap
    
class World:
    
    def __init__(self,origin=(0,0),region_size=(50,50)):
        self._regions = dict()
        self._ways = dict()
        self._origin = origin
        self._size = region_size
    
    def region_id(self,position):
        return floor((position[0]-self._origin[0])/self._size[0]),floor((position[1]-self._origin[1])//self._size[1])
    
    def add_region(self,region_id):
        if region_id not in self._regions:
            self._regions[region_id] = Region()
            self._ways[region_id] = set()
    
    def connect_regions(self,region_id1,region_id2):
        if region_id1 != region_id2:
            self._ways[region_id1].add(region_id2)
            self._ways[region_id2].add(region_id1)


######
import cv2

def viz_cmap(cmap,size=None):
    r,c = cmap.shape
    x = np.full((r,c,3),128,np.uint8)
    for i in range(r):
        for j in range(c):
            if cmap[i,j] == 0:
                x[i,j] = [0,0,0]
            elif cmap[i,j] == 1:
                x[i,j] = [0,128,255]
            elif cmap[i,j] == 2:
                x[i,j] = [255,255,0]
            elif cmap[i,j] == 3:
                x[i,j] = [255,255,255]
    if size is not None:
        x = cv2.resize(x,size)
    cv2.imshow('CMap',x)

def F(n,s): return s/n

def color(n,s,M):
    f = F(n,s)/M
    return 255-200*f,255-200*f,255-200*f
    a,b,c = 255,255,255
    r,g,b = a*f*f*f,b*f*f,c*f
    return (b,g,r)

def viz_costmap(kmap,start,size=None):
    r,c = len(kmap),len(kmap[0])
    x = np.zeros((r,c,3),np.uint8)
    M = 0
    for i in range(r):
        for j in range(c):
            n,s = kmap[i][j]
            if n!= 0:
                M = max(F(n,s),M)
    for i in range(r):
        for j in range(c):
            n,s = kmap[i][j]
            if n != 0:
                x[i,j] = color(n,s,M)
    x = cv2.circle(x,start,3,(0,0,255),cv2.FILLED)
    if size is not None:
        x = cv2.resize(x,size)
    cv2.imshow('KMap',x)

def close_cv2():
    cv2.destroyAllWindows() 

def main():
    from auxiliary import load_critical_map
    cmap = load_critical_map('/home/ghost/Desktop/dev_ws/src/controller/controller/saved_cmaps/cmap_386089.npy')
    print(cmap.shape)
    r = Region(cmap,0.5,2.0)
    viz_cmap(cmap)#,(500,500))
    start = (50,27)
    kmap = r.find_unexplored_subregion(start,0)
    viz_costmap(kmap,start)#,(500,500))
    while cv2.waitKey(100) <= 0: pass
    close_cv2()

main()