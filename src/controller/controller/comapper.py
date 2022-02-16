import cv2
from math import acos, atan2, ceil, cos, floor, pi, sin, sqrt
from queue import PriorityQueue
import numpy as np
from numpy.random.mtrand import randint

class GridTree:
    def __init__(self,cost):
        self.cost = cost
        self.north,self.south,self.west,self.east,self.north_east,self.north_west,self.south_east,self.south_west = None,None,None,None,None,None,None,None

class Region:
    
    def __init__(self,critical_map,resolution,sensor_radius):
        self._cmap = critical_map
        self._costmap = {'hypercritical':100,'radian':10,'meter':1}
        self._infinity = 2**31
        self._resolution = resolution
        self._sensor_radius = sensor_radius
        self._radius = floor(self._sensor_radius/self._resolution)
        self._rows,self._cols = self._cmap.shape

    def _neighbour_dist(self,cell,neighbour):
        if cell[0]-neighbour[0] and cell[1]-neighbour[1]: return sqrt(2)
        return 1
    
    def _angle_diff(self,parents_parent,parent,child):
        diff = (parent[0]-parents_parent[0])*(child[0]-parent[0])+(parent[1]-parents_parent[1])*(child[1]-parent[1])
        return acos(diff/(self._neighbour_dist(parents_parent,parent)*self._neighbour_dist(parent,child)))

    def _movement_cost(self,parent_cell,child_cell,parents_parent_cell,initial_yaw):
        ang = 0
        if parents_parent_cell is not None:
            ang = self._angle_diff(parents_parent_cell,parent_cell,child_cell)
        else:
            ang = cos(initial_yaw)*(child_cell[1]-parent_cell[1])+sin(initial_yaw)*(parent_cell[0]-child_cell[0])
            ang /= self._neighbour_dist(parent_cell,child_cell)
            ang = acos(ang)
        return self._costmap['radian']*ang + self._costmap['meter']*self._neighbour_dist(parent_cell,child_cell)

    def find_unexplored_goal(self,start_cell,initial_yaw):

        if start_cell[0] < 0 or start_cell[0] >= self._rows or start_cell[1] < 0 or start_cell[1] >= self._cols or self._cmap[start_cell] in [0,1]:
            return None

        frontier = PriorityQueue()
        explored = set()
        cache = [[(self._infinity,None) for j in range(self._cols)] for i in range(self._rows)] #movement_cost,parent

        cache[start_cell[0]][start_cell[1]] = (0,None)
        frontier.put((0,start_cell))

        while not frontier.empty():
            
            m,(i,j) = frontier.get()
            m,p = cache[i][j]
            explored.add((i,j))
            
            for x,y in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i+1,j+1),(i+1,j-1),(i-1,j+1),(i-1,j-1)]:
                if x >= 0 and y >= 0 and x < self._rows and y < self._cols and (x,y) not in explored and self._cmap[x,y] not in [0,1]:
                    
                    m2 = m + self._movement_cost((i,j),(x,y),p,initial_yaw)

                    if self._cmap[x,y] == 2:
                        m2 += self._costmap['hypercritical']
                    
                    if cache[x][y][0] > m2:
                        frontier.put((m2,(x,y)))
                        cache[x][y] = (m2,(i,j))
        
        children = [[set() for j in range(self._cols)] for i in range(self._rows)]
        for i in range(self._rows):
            for j in range(self._cols):
                _,p = cache[i][j]
                if p is not None:
                    children[p[0]][p[1]].add((i,j))
        
        coverage = self._coverage(start_cell,children,cache)

        goal = None
        cov = self._infinity

        for i in range(self._rows):
            for j in range(self._cols):
                if coverage[i][j] < cov:
                    goal = (i,j)
                    cov = coverage[i][j]

        return goal

    def _coverage_function(self,n,c):
        if n == 0: return self._infinity
        return sqrt(c)/n
    
    def _coverage(self,start,ways,costs):
        
        def right_strip(i,j,cache):
            y = j+1+self._radius
            strip = set()
            if y>= 0 and y < self._cols:
                for x in range(max(i-self._radius,0),min(i+self._radius+1,self._rows)):
                    if (x,y) not in cache:
                        strip.add((x,y))
            return strip

        def left_strip(i,j,cache):
            y = j-1-self._radius
            strip = set()
            if y>= 0 and y < self._cols:
                for x in range(max(i-self._radius,0),min(i+self._radius+1,self._rows)):
                    if (x,y) not in cache:
                        strip.add((x,y))
            return strip
        
        def down_strip(i,j,cache):
            x = i+1+self._radius
            strip = set()
            if x < self._rows:
                for y in range(max(j-self._radius,0),min(j+self._radius+1,self._cols)):
                    if (x,y) not in cache:
                        strip.add((x,y))
            return strip

        def up_strip(i,j,cache):
            x = i-1-self._radius
            strip = set()
            if x>= 0 and x < self._rows:
                for y in range(max(j-self._radius,0),min(j+self._radius+1,self._cols)):
                    if (x,y) not in cache:
                        strip.add((x,y))
            return strip
        
        def down_right_strip(i,j,cache):
            root_r = floor(self._radius*sqrt(2))
            strip = set()
            for v in range(j,min(j+root_r+1,self._cols-1)):
                u = -v+i+j+root_r
                if u < self._rows and self._cmap[u][v+1] == 4 and (u,v+1) not in cache:
                    strip.add((u,v+1))
                if u+1 < self._rows and self._cmap[u+1][v+1] == 4 and (u+1,v+1) not in cache:
                    strip.add((u+1,v+1))
            return strip
        
        def down_left_strip(i,j,cache):
            root_r = floor(self._radius*sqrt(2))
            strip = set()
            for v in range(max(1,j-root_r),j+1):
                u = v+i-j+root_r
                if u < self._rows and self._cmap[u][v-1] == 4 and (u,v-1) not in cache:
                    strip.add((u,v-1))
                if u+1 < self._rows and self._cmap[u+1][v-1] == 4 and (u+1,v-1) not in cache:
                    strip.add((u+1,v-1))
            return strip

        def up_right_strip(i,j,cache):
            root_r = floor(self._radius*sqrt(2))
            strip = set()
            for v in range(j,min(j+root_r+1,self._cols-1)):
                u = v+i-j-root_r
                if u >= 0 and self._cmap[u][v+1] == 4 and (u,v+1) not in cache:
                    strip.add((u,v+1))
                if u-1 >= 0 and self._cmap[u-1][v+1] == 4 and (u-1,v+1) not in cache:
                    strip.add((u-1,v+1))
            return strip

        def up_left_strip(i,j,cache):
            root_r = floor(self._radius*sqrt(2))
            strip = set()
            for v in range(max(1,j-root_r),j+1):
                u = -v+i+j-root_r
                if u >= 0 and self._cmap[u][v-1] == 4 and (u,v-1) not in cache:
                    strip.add((u,v-1))
                if u-1 >= 0 and self._cmap[u-1][v-1] == 4 and (u-1,v-1) not in cache:
                    strip.add((u-1,v-1))
            return strip

        coverage = [[self._infinity for j in range(self._cols)] for i in range(self._rows)]
      
        def dfs(root,cache):
            i,j = root
            coverage[i][j] = self._coverage_function(len(cache),costs[i][j][0])
            for x,y in ways[i][j]:
                if x>i:
                    if y>j:
                        s = down_right_strip(i,j,cache)
                    elif y==j:
                        s = down_strip(i,j,cache)
                    else:
                        s = down_left_strip(i,j,cache)
                elif x==i:
                    if y>j:
                        s = right_strip(i,j,cache)
                    else:
                        s = left_strip(i,j,cache)
                else:
                    if y>j:
                        s = up_right_strip(i,j,cache)
                    elif y==j:
                        s = up_strip(i,j,cache)
                    else:
                        s = up_left_strip(i,j,cache)
                for e in s:
                    cache.add(e)
                dfs((x,y),cache)
                for e in s:
                    cache.remove(e)

        cache = set()
        dfs(start,cache)

        return coverage


class World:


    def __init__(self,store,origin=(0,0),region_size=(50,50)):
        self._regions = dict()
        self._local_equivalences = dict()
        self._cross_equivalences = dict()
        self._store = store
        self._origin = origin
        self._size = region_size
        self._resolution = store.get('general','map_resolution')
        self._sensor_radius = store.get('general','sensor_radius')
        self._critical_radius = store.get('general','critical_radius')
        self._hypercritical_radius = store.get('general','hyper_critical_radius')
        self._obstacle_radius = store.get('general','map_obstacle_lookup_radius')
        self._costs = store.get('general','planning_cost_map')
        self._color_cache = {0:(0,0,0)}


    def region_id(self,position):
        return floor((position[0]-self._origin[0])/self._size[0]),floor((position[1]-self._origin[1])/self._size[1])


    def get_region(self,position):
        return self.get_region_by_id(self.region_id(position))


    def get_region_by_id(self,r):
        if r not in self._regions:
            b = (self._origin[0]+r[0]*self._size[0],self._origin[1]+r[1]*self._size[1],self._origin[0]+(r[0]+1)*self._size[0],self._origin[1]+(r[1]+1)*self._size[1])
            matrix_shape = (ceil((b[3]-b[1])/self._resolution),ceil((b[2]-b[0])/self._resolution))
            self._regions[r] = (b,np.full(matrix_shape,4,np.uint8))
        return self._regions[r]


    def current_region_id(self):
        return self.region_id(self._store.get('estimator','position'))

    def current_region(self):
        return self.get_region(self._store.get('estimator','position'))


    def current_map(self):
        return self.current_region()[1]


    def current_regional_bounds(self):
        return self.current_region[0]


    def regional_bounds(self,position):
        return self.get_region(position)[0]


    def regional_bounds_by_id(self,region_id):
        return self.get_region_by_id(region_id)[0]


    def regional_transform(self,bounds,cell):
        return bounds[0]+(cell[1]+0.5)*self._resolution,bounds[3]-(cell[0]+0.5)*self._resolution


    def inverse_regional_transform(self,bounds,position):
        return floor((bounds[3]-position[1])/self._resolution),floor((position[0]-bounds[0])/self._resolution)


    def patch(self,patch):

        cx,cy,ct = patch.__origin__
        r = self.region_id((cx,cy))
        rb = self.regional_bounds_by_id(r)
        pb = cx-self._sensor_radius-2*self._hypercritical_radius,cy-self._sensor_radius-2*self._hypercritical_radius,cx+self._sensor_radius+2*self._hypercritical_radius,cy+self._sensor_radius+2*self._hypercritical_radius

        #Local Patching
        if rb[0] < pb[0] and rb[2] > pb[2] and rb[1] < pb[1] and rb[3] > pb[3]:
            self.local_patch(self.get_region_by_id(r),patch)
            return
        
        #Boundary Patching
        neighbouring_regions = dict()
        for i in [-1,0,1]:
            for j in [-1,0,1]:
                r2 = self.get_region_by_id((r[0]+i,r[1]+j))
                rb = r2[0]
                if pb[0] <= rb[2] and rb[0] <= pb[2] and pb[1] <= rb[3] and rb[1] <= pb[3]:
                    neighbouring_regions[(r[0]+i,r[1]+j)] = r2
        self.boundary_patch(neighbouring_regions,patch)


    def boundary_patch(self,regions,patch):

        sensor_radius = self._sensor_radius
        critical_radius = self._critical_radius
        hypercritical_radius = self._hypercritical_radius
        obstacle_radius = self._obstacle_radius
        resolution = self._resolution

        cx,cy,ct = patch.__origin__
        
        pb = cx-sensor_radius,cy-sensor_radius,cx+sensor_radius,cy+sensor_radius        

        pom,po = patch.__obstacle_map__,patch.__obstacles__
        j=0
        poa = []
        for i in range(pom.__length__):
            if pom.bit_get(i) == 1 and po[j] <= sensor_radius:
                poa.append(po[j])
                j += 1
            else:
                poa.append(None)

        for region in regions.values():
    
            cb,cm = region
            cr,cc = cm.shape
            kb = max(pb[0],cb[0]),max(pb[1],cb[1]),min(pb[2],cb[2]),min(pb[3],cb[3])
            kr,kc = ceil((kb[3]-kb[1])/resolution),ceil((kb[2]-kb[0])/resolution)
            u0,v0 = floor((cb[3]-(kb[3]-0.5*resolution))/resolution),floor((kb[0]+0.5*resolution-cb[0])/resolution)
            imin,jmin,imax,jmax = max(-u0,0),max(-v0,0),min(u0+kr,cr)-u0,min(v0+kc,cc)-v0
            y = kb[3]+resolution/2
            
            for i in range(imin,imax):
                y -= resolution
                x = kb[0]-resolution/2
                for j in range(jmin,jmax):
                    x += resolution
                    r = sqrt((x-cx)*(x-cx)+(y-cy)*(y-cy))
                    t = atan2(y-cy,x-cx)+pi/2-ct
                    if t < -pi: t += 2*pi
                    if t > pi: t -= 2*pi
                    if r <= sensor_radius and t >= 0 and t < pi:
                        k = floor(180*t/pi)
                        p = poa[k]
                        u = u0+i
                        v = v0+j
                        if p is not None:
                            if r < p:
                                cm[u,v] = 3
                            elif r <= p+obstacle_radius:
                                cm[u,v] = 0
                        else:
                            cm[u,v] = 3

        xmin,ymax = cx-sensor_radius-3*hypercritical_radius ,cy+sensor_radius+3*hypercritical_radius
        begin_region_id = self.region_id((xmin,ymax))
        imin,jmin = self.inverse_regional_transform(regions[begin_region_id][0],(xmin,ymax))
        hrad = ceil(hypercritical_radius/resolution)
        crad = ceil(critical_radius/resolution)
        srad = ceil(sensor_radius/resolution)
        imax = imin+2*srad+4*hrad
        jmax = jmin+2*srad+4*hrad
        cache_size = 4*hrad+2*srad+1
        twice_hrad = 2*hrad
        twice_hrad_plus_1 = twice_hrad+1
        twice_crad = 2*crad
        twice_crad_plus_1 = twice_crad+1
        imin_bound = imin+twice_hrad
        jmin_bound = jmin+twice_hrad
        cr,cc = regions[begin_region_id][1].shape

        def get_cell_value(i,j):
            return regions[(begin_region_id[0]+floor(j/cc),begin_region_id[1]-floor(i/cr))][1][i%cr,j%cc]

        def set_cell_value(i,j,k):
            cm = regions[(begin_region_id[0]+floor(j/cc),begin_region_id[1]-floor(i/cr))][1]
            p =i%cr,j%cc
            if cm[p] > k: cm[p] = k

        cache = [[0,0] for _ in range(cache_size)]
        for i in range(imin,imax):
            end_hyper_critical_index = 0
            end_critical_index = 0
            for j in range(jmin,jmax):
                if get_cell_value(i+hrad,j+hrad) == 0:
                    cache[j-jmin][0] = twice_hrad_plus_1
                if get_cell_value(i+crad,j+crad) == 0:
                    cache[j-jmin][1] = twice_crad_plus_1
                if cache[j-jmin][0] > 0:
                    end_hyper_critical_index = j+twice_hrad_plus_1
                    cache[j-jmin][0] -= 1
                if cache[j-jmin][1] > 0:
                    end_critical_index = j+twice_crad_plus_1
                    cache[j-jmin][1] -= 1
                if i >= imin_bound and j >= jmin_bound:
                    if j < end_critical_index:
                        set_cell_value(i,j,1)
                    elif j < end_hyper_critical_index:
                        set_cell_value(i,j,2)


    def local_patch(self,region,patch):

        cb,cm = region
        sensor_radius = self._sensor_radius
        critical_radius = self._critical_radius
        hypercritical_radius = self._hypercritical_radius
        obstacle_radius = self._obstacle_radius
        resolution = self._resolution

        cx,cy,ct = patch.__origin__
        
        cr,cc = cm.shape
        pb = cx-sensor_radius,cy-sensor_radius,cx+sensor_radius,cy+sensor_radius        
        pr,pc = ceil((pb[3]-pb[1])/resolution),ceil((pb[2]-pb[0])/resolution)

        pom,po = patch.__obstacle_map__,patch.__obstacles__
        j=0
        poa = []
        for i in range(pom.__length__):
            if pom.bit_get(i) == 1 and po[j] <= sensor_radius:
                poa.append(po[j])
                j += 1
            else:
                poa.append(None)

        u0,v0 = floor((cb[3]-(pb[3]-0.5*resolution))/resolution),floor((pb[0]+0.5*resolution-cb[0])/resolution)
        imin,jmin,imax,jmax = max(-u0,0),max(-v0,0),min(u0+pr,cr)-u0,min(v0+pc,cc)-v0
        y = pb[3]+resolution/2

        for i in range(imin,imax):
            y -= resolution
            x = pb[0]-resolution/2
            for j in range(jmin,jmax):
                x += resolution
                r = sqrt((x-cx)*(x-cx)+(y-cy)*(y-cy))
                t = atan2(y-cy,x-cx)+pi/2-ct
                if t < -pi: t += 2*pi
                if t > pi: t -= 2*pi
                if r <= sensor_radius and t >= 0 and t < pi:
                    k = floor(180*t/pi)
                    p = poa[k]
                    u = u0+i
                    v = v0+j
                    if p is not None:
                        if r < p:
                            cm[u,v] = 3
                        elif r <= p+obstacle_radius:
                            cm[u,v] = 0
                    else:
                        cm[u,v] = 3

        imin = floor((cb[3]-cy-sensor_radius-3*hypercritical_radius)/resolution)
        jmin = floor((cx-sensor_radius-3*hypercritical_radius-cb[0])/resolution)
        hrad = ceil(hypercritical_radius/resolution)
        crad = ceil(critical_radius/resolution)
        srad = ceil(sensor_radius/resolution)
        imax = imin+2*srad+4*hrad
        jmax = jmin+2*srad+4*hrad
        cache_size = 4*hrad+2*srad+1
        twice_hrad = 2*hrad
        twice_hrad_plus_1 = twice_hrad+1
        twice_crad = 2*crad
        twice_crad_plus_1 = twice_crad+1
        imin_bound = imin+twice_hrad
        jmin_bound = jmin+twice_hrad

        cache = [[0,0] for _ in range(cache_size)]

        for i in range(imin,imax):
            end_hyper_critical_index = 0
            end_critical_index = 0
            for j in range(jmin,jmax):
                if cm[i+hrad,j+hrad] == 0:
                    cache[j-jmin][0] = twice_hrad_plus_1
                if cm[i+crad,j+crad] == 0:
                    cache[j-jmin][1] = twice_crad_plus_1
                if cache[j-jmin][0] > 0:
                    end_hyper_critical_index = j+twice_hrad_plus_1
                    cache[j-jmin][0] -= 1
                if cache[j-jmin][1] > 0:
                    end_critical_index = j+twice_crad_plus_1
                    cache[j-jmin][1] -= 1
                if i >= imin_bound and j >= jmin_bound:
                    if j < end_critical_index and cm[i,j] > 1:
                        cm[i,j] = 1
                    elif j < end_hyper_critical_index and cm[i,j] > 2:
                        cm[i,j] = 2

        
    def local_plan(self,region,start_cell,goal_cell,initial_yaw):
        
        def ndist(p,c):
            dx = p[0]-c[0]
            dy = p[1]-c[1]
            return sqrt(dx*dx+dy*dy)
        
        def dang(pp,p,c):
            diff = (p[0]-pp[0])*(c[0]-p[0])+(p[1]-pp[1])*(c[1]-p[1])
            return acos(diff/(ndist(pp,p)*ndist(p,c)))

        def G(p,c,pp):
            ang = 0
            if pp is not None:
                ang = dang(pp,p,c)
            else:
                ang = cos(initial_yaw)*(c[1]-p[1])+sin(initial_yaw)*(p[0]-c[0])
                ang /= ndist(p,c)
                ang = acos(ang)
            
            return cost_per_radian*ang + H(p,c)
        
        def H(s,g):
            dx = s[0]-g[0]
            dy = s[1]-g[1]
            return cost_per_meter*sqrt(dx*dx+dy*dy)

        _,obstacles = region
        cost_hyper_critical = self._costs['hypercritical']
        cost_per_radian = self._costs['angular']
        cost_per_meter = self._costs['metric']

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

        initial_cost = H(start_cell,goal_cell)
        frontier.put((initial_cost,start_cell))
        cost_map[start_cell] = (initial_cost,0)
        parent_map[start_cell] = None

        while not frontier.empty():
            f_cost,(i,j) = frontier.get()
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
                if x >= 0 and y >= 0 and x < rows and y < cols and (x,y) not in explored and obstacles[x,y] > 1:
                    g = g_cost + G((i,j),(x,y),parent_map[(i,j)])
                    if obstacles[x,y] == 2:
                        g += cost_hyper_critical
                    if (x,y) not in cost_map or cost_map[(x,y)][1] > g:
                        f = H((x,y),goal_cell) + g
                        frontier.put((f,(x,y)))
                        cost_map[(x,y)] = (f,g)
                        parent_map[(x,y)] = (i,j)
        
        return []


    def reduce_plan(self,plan):
        if len(plan) < 3: return plan
        reduced_plan = [plan[0]]
        old_direction = (plan[1][0]-plan[0][0],plan[1][1]-plan[0][1])
        for i in range(2,len(plan)):
            new_direction = (plan[i][0]-plan[i-1][0],plan[i][1]-plan[i-1][1])
            if old_direction != new_direction:
                reduced_plan.append(plan[i-1])
                old_direction = new_direction
        reduced_plan.append(plan[-1])
        return reduced_plan
    

    def transform_plan(self,region_bounds,plan):
        plan_tf = []
        for cell in plan:
            plan_tf.append(self.regional_transform(region_bounds,cell))
        return plan_tf
    

    def verify_plan(self,region,plan):
        _,obstacles = region
        for cell in plan:
            if obstacles[cell] < 2:
                return False
        return True
    

    def compare_region_ids(self,id1,id2):
        if id1[0] > id2[0]: return 1
        elif id1[0] < id2[0]: return -1
        elif id1[1] > id2[1]: return 1
        elif id1[1] < id2[0]: return -1
        return 0
    

    def cache_inter_regional_paths_by_id(self,region_id):
        leq = self._compute_local_equivalences_by_id(region_id)
        self._local_equivalences[region_id] = leq
        i,j = region_id
        for region_id2 in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i-1,j-1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]:
            if region_id2 in self._local_equivalences:
                leq2 = self._local_equivalences[region_id2]
                if self.compare_region_ids(region_id,region_id2) > 0:
                    self._cross_equivalences[(region_id,region_id2)] = self._compute_cross_equivalences_by_id(region_id,region_id2,leq,leq2)
                else:
                    self._cross_equivalences[(region_id2,region_id)] = self._compute_cross_equivalences_by_id(region_id2,region_id,leq2,leq)


    def _compute_local_equivalences_by_id(self,region_id):
        
        _,map = self.get_region_by_id(region_id)
        cost_hyper_critical = self._costs['hypercritical']
        cost_per_radian = self._costs['angular']
        cost_per_meter = self._costs['metric']
        rows,cols = map.shape
        stack = list()
        seq = 1
        local_equivalence_map = [[0 for j in range(cols)] for i in range(rows)]
        centroids = [None]
        bases = [None]
        costs = [None]
        boundaries = [None]
        path_tree = dict()

        for x in range(rows):
            for y in range(cols):
                if map[x,y] > 1 and local_equivalence_map[x][y] == 0:
                    stack.append((x,y))
                    centroid = [0,0]
                    n = 0
                    while stack:
                        i,j = stack.pop()
                        local_equivalence_map[i][j] = seq
                        centroid[0] += i
                        centroid[1] += j
                        n += 1
                        for k,l in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i-1,j-1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]:
                            if k >= 0 and k < rows and l >= 0 and l < cols and map[k,l] > 1 and local_equivalence_map[k][l] == 0:
                                stack.append((k,l))
                    centroids.append((centroid[0]/n+0.5,centroid[1]/n+0.5))
                    bases.append(None)
                    costs.append(None)
                    boundaries.append(list())
                    seq += 1

        for i in range(rows):
            for j in range(cols):
                c = local_equivalence_map[i][j]
                if c > 0:
                    centroid = centroids[c]
                    di = centroid[0]-i
                    dj = centroid[1]-j
                    cost = sqrt(di*di+dj*dj)
                    if map[i,j] == 2:
                        cost += cost_hyper_critical
                    if bases[c] is None or cost < costs[c]:
                        bases[c] = (i,j)
                        costs[c] = cost
        
        for i in range(rows):
            l = local_equivalence_map[i] 
            if l[0] > 0:
                boundaries[l[0]].append((i,0))
            if l[cols-1] > 0:
                boundaries[l[cols-1]].append((i,cols-1))
        
        for j in range(cols):
            if local_equivalence_map[0][j] > 0:
                boundaries[local_equivalence_map[0][j]].append((0,j))
            if local_equivalence_map[rows-1][j] > 0:
                boundaries[local_equivalence_map[rows-1][j]].append((rows-1,j))


        def ndist(p,c):
            dx = p[0]-c[0]
            dy = p[1]-c[1]
            return sqrt(dx*dx+dy*dy)
        
        def dang(pp,p,c):
            diff = (p[0]-pp[0])*(c[0]-p[0])+(p[1]-pp[1])*(c[1]-p[1])
            return acos(diff/(ndist(pp,p)*ndist(p,c)))

        def H(s,g):
            dx = s[0]-g[0]
            dy = s[1]-g[1]
            return cost_per_meter*sqrt(dx*dx+dy*dy)

        def G(p,c,pp):
            if pp is not None:
                return cost_per_radian*dang(pp,p,c) + H(p,c)
            return H(p,c)


        for c in range(1,len(bases)):

            b = bases[c]
            boundary_cells = boundaries[c]
            frontier = PriorityQueue()
            explored = set()
            cache = dict()
            frontier.put((0,b))
            cache[b] = (0,None)
            
            while not frontier.empty():
                cost,(i,j) = frontier.get()
                cost,parent = cache[(i,j)]
                explored.add((i,j))
                for x,y in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i-1,j-1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]:
                    if x >= 0 and y >= 0 and x < rows and y < cols and (x,y) not in explored and map[x,y] > 1:
                        child_cost = cost + G((i,j),(x,y),parent)
                        if map[x,y] == 2:
                            child_cost += cost_hyper_critical
                        if (x,y) not in cache or cache[(x,y)][0] > child_cost:
                            frontier.put((child_cost,(x,y)))
                            cache[(x,y)] = (child_cost,(i,j))
            
            tree_cache = dict()
            stack = list()

            for bcell in boundary_cells:
                stack.append(bcell)
                tree_cache[bcell] = [bcell,None]
                path_tree[bcell] = (tree_cache[bcell],bases[c],cache[bcell][0])

            while stack:
                cell = stack.pop()
                parent_cell = cache[cell][1]
                if parent_cell is not None:
                    if parent_cell not in tree_cache:
                        tree_cache[parent_cell] = [parent_cell,None]
                        stack.append(parent_cell)
                    tree_cache[cell][1] = tree_cache[parent_cell]
        
        bases = {base for base in bases}
        
        return bases,path_tree


    def _compute_cross_equivalences_by_id(self,region_id1,region_id2,local_equivalence1,local_equivalence2):
        
        d = region_id1[1]-region_id2[1],region_id1[0]-region_id2[0]
        _,r1 = self.get_region_by_id(region_id1)
        _,r2 = self.get_region_by_id(region_id2)
        _,p1 = local_equivalence1
        _,p2 = local_equivalence2
        row1,col1 = r1.shape
        row2,col2 = r2.shape
        cross_equivalence_map = dict()


        def update_crossmap(i1,j1,i2,j2):
            x1,base1,cost1 = p1[(i1,j1)]
            x2,base2,cost2 = p2[(i2,j2)]
            cost = cost1+cost2
            if base1 not in cross_equivalence_map:
                cross_equivalence_map[base1] = dict()
            cm1 = cross_equivalence_map[base1]
            if base2 not in cm1 or cm1[base2][2] > cost:
                cm1[base2] = (x1,x2,cost)
        
        def fill_crossmap_column():
            if d[1]>0:
                j1,j2=0,col2-1
            else:
                j1,j2= col1-1,0            
            for i1 in range(row1):
                if r1[i1,j1] > 1:
                    for i2 in range(i1-1,i1+2):
                        if i2>=0 and i2<row2 and r2[i2,j2] > 1:
                            update_crossmap(i1,j1,i2,j2)

        def fill_crossmap_row():
            if d[0]>0:
                i1,i2=row1-1,0
            else:
                i1,i2= 0,row2-1            
            for j1 in range(col1):
                if r1[i1,j1] > 1:
                    for j2 in range(j1-1,j1+2):
                        if j2>=0 and j2<col2 and r2[i2,j2] > 1:
                            update_crossmap(i1,j1,i2,j2)
        
        def fill_crossmap_corner():
            if d[0] == 1:
                i1,i2 = row1-1,0
            else:
                i1,i2 = 0,row2-1
            if d[1] == 1:
                j1,j2 = col1-1,0
            else:
                j1,j2 = 0,col2-1
            if r1[i1,j1] > 1 and r2[i2,j2] > 1:
                update_crossmap(i1,j1,i2,j2)


        if d in [(0,1),(0,-1)]:
            fill_crossmap_column()
        elif d in [(1,0),(-1,0)]:
            fill_crossmap_row()
        elif d in [(-1,-1),(-1,1),(1,-1),(1,1)]:
            fill_crossmap_corner()
        else:
            return None
        
        return cross_equivalence_map


    def cache_inter_regional_paths(self):
        self.cache_inter_regional_paths_by_id(self.current_region_id())


    def find_centroid(self,region,centroids,start_cell):
   
        def ndist(p,c):
            dx = p[0]-c[0]
            dy = p[1]-c[1]
            return sqrt(dx*dx+dy*dy)
        
        def dang(pp,p,c):
            diff = (p[0]-pp[0])*(c[0]-p[0])+(p[1]-pp[1])*(c[1]-p[1])
            return acos(diff/(ndist(pp,p)*ndist(p,c)))

        def G(p,c,pp):
            if pp is not None:
                return cost_per_radian*dang(pp,p,c) + H(p,c)
            return H(p,c)            
        
        def H(s,g):
            dx = s[0]-g[0]
            dy = s[1]-g[1]
            return cost_per_meter*sqrt(dx*dx+dy*dy)

        _,obstacles = region
        cost_hyper_critical = self._costs['hypercritical']
        cost_per_radian = self._costs['angular']
        cost_per_meter = self._costs['metric']
        rows,cols = obstacles.shape

        if start_cell[0] < 0 or start_cell[0] >= rows or start_cell[1] < 0 or start_cell[1] >= cols or obstacles[start_cell[0]][start_cell[1]] < 2:
            return None

        frontier = PriorityQueue()
        explored = set()
        cost_map = dict()

        frontier.put((0,start_cell))
        cost_map[start_cell] = [0,None]

        while not frontier.empty():
            g_cost,(i,j) = frontier.get()
            g_cost,parent = cost_map[(i,j)]
            explored.add((i,j))

            if (i,j) in centroids:
                cell = (i,j)
                path = []
                while cell is not None:
                    path.append(cell)
                    cell = cost_map[cell][1]
                path.reverse()
                return (i,j),path
            
            for x,y in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i-1,j-1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]:
                if x >= 0 and y >= 0 and x < rows and y < cols and (x,y) not in explored and obstacles[x,y] > 1:
                    g = g_cost + G((i,j),(x,y),parent)
                    if obstacles[x,y] == 2:
                        g += cost_hyper_critical
                    if (x,y) not in cost_map or cost_map[(x,y)][0] > g:
                        frontier.put((g,(x,y)))
                        cost_map[(x,y)] = [g,(i,j)]
        
        return None


    def global_plan(self,start_position,goal_position,initial_yaw):

        id1 = self.region_id(start_position)
        id2 = self.region_id(goal_position)
        r1 = self.get_region_by_id(id1)
        r2 = self.get_region_by_id(id2)

        start_cell = self.inverse_regional_transform(r1[0],start_position)
        goal_cell = self.inverse_regional_transform(r2[0],goal_position)

        if id1 == id2:
            plan = self.local_plan(r1,start_cell,goal_cell,initial_yaw)
            if len(plan) > 0:
                global_plan = []
                for cell in plan:
                    global_plan.append((id1,cell))
                return global_plan
        
        if id1 not in self._local_equivalences or id2 not in self._local_equivalences:
            return []
        
        c1,p1 = self._local_equivalences[id1]
        c2,p2 = self._local_equivalences[id2]

        start_centroid = self.find_centroid(r1,c1,start_cell)
        if start_centroid is None: return []
        goal_centroid = self.find_centroid(r2,c2,goal_cell)
        if goal_centroid is None: return []

        region_width,region_height = self._size
        
        def H(s,g):
            dx = region_width*(g[0][0]-s[0][0])+self._resolution*(g[1][1]-s[1][1])
            dy = region_height*(g[0][1]-s[0][1])+self._resolution*(s[1][0]-g[1][0])
            return cost_per_meter*sqrt(dx*dx+dy*dy)

        cost_per_meter = self._costs['metric']

        frontier = PriorityQueue()
        explored = set()
        cache = dict()

        start = (id1,start_centroid[0])
        goal = (id2,goal_centroid[0])
        initial_cost = H(start,goal)
        frontier.put((initial_cost,start))
        cache[start] = (0,None)

        while not frontier.empty():
            f_cost,node = frontier.get()
            g_cost,parent = cache[node]
            explored.add(node)
            if node == goal:
                current_node = goal
                parent_node = cache[current_node][1]
                global_plan = []
                for cell in goal_centroid[1]:
                    global_plan.append((id2,cell))
                while parent_node is not None:
                    if self.compare_region_ids(parent_node[0],current_node[0])>0:
                        x1,x2,_ = self._cross_equivalences[(parent_node[0],current_node[0])][parent_node[1]][current_node[1]]
                    else:
                        x2,x1,_ = self._cross_equivalences[(current_node[0],parent_node[0])][current_node[1]][parent_node[1]]
                    cell = x2
                    subplan = []
                    while cell is not None:
                        subplan.append((current_node[0],cell[0]))
                        cell = cell[1]
                    subplan.reverse()
                    global_plan.extend(subplan)
                    cell = x1
                    while cell is not None:
                        global_plan.append((parent_node[0],cell[0]))
                        cell = cell[1]
                    current_node = parent_node
                    parent_node = cache[current_node][1]
                for cell in reversed(start_centroid[1]):
                    global_plan.append((id1,cell))
                global_plan.reverse()
                return global_plan
            i,j = node[0]
            for rid in [(i-1,j),(i+1,j),(i,j-1),(i,j+1),(i-1,j-1),(i+1,j+1),(i-1,j+1),(i+1,j-1)]:
                if self.compare_region_ids(node[0],rid) > 0:
                    if (node[0],rid) in self._cross_equivalences:
                        x = self._cross_equivalences[(node[0],rid)]
                        if node[1] in x:
                            for c,k in x[node[1]].items():
                                if (rid,c) not in explored:
                                    g_cost2 = g_cost + k[2]
                                    if (rid,c) not in cache or cache[(rid,c)][0] > g_cost2:
                                        f_cost2 = g_cost2 + H((rid,c),goal)
                                        frontier.put((f_cost2,(rid,c)))
                                        cache[(rid,c)] = (g_cost2,node)
                else:
                    if (rid,node[0]) in self._cross_equivalences:
                        for c,m in self._cross_equivalences[(rid,node[0])].items():
                            if node[1] in m:
                                k = m[node[1]]
                                if (rid,c) not in explored:
                                    g_cost2 = g_cost + k[2]
                                    if (rid,c) not in cache or cache[(rid,c)][0] > g_cost2:
                                        f_cost2 = g_cost2 + H((rid,c),goal)
                                        frontier.put((f_cost2,(rid,c)))
                                        cache[(rid,c)] = (g_cost2,node)
        
        return []


    def reduce_global_plan(self, global_plan):
        if len(global_plan) < 3: return global_plan

        rows = ceil(self._size[1]/self._resolution)
        cols = ceil(self._size[0]/self._resolution)
        

        def direction(parent,child):
            r1,(i1,j1) = parent
            r2,(i2,j2) = child
            dx = r2[0]-r1[0]
            dy = r2[1]-r2[1]
            if dx>0:
                j1 = -1
            elif dx<0:
                j1 = cols
            if dy>0:
                i1 = rows
            elif dy<0:
                i1 = -1
            return (i2-i1,j2-j1)
            

        reduced_global_plan = [global_plan[0]]
        old_direction = direction(global_plan[0],global_plan[1])
        for i in range(2,len(global_plan)):
            new_direction = direction(global_plan[i-1],global_plan[i])
            if old_direction != new_direction:
                if reduced_global_plan[-1] != global_plan[i-1]:
                    reduced_global_plan.append(global_plan[i-1])
                if new_direction != (0,0):
                    old_direction = new_direction
        
        if reduced_global_plan[-1] != global_plan[-1]:
            reduced_global_plan.append(global_plan[-1])

        return reduced_global_plan
    
    
    def transform_global_plan(self,global_plan):
        global_plan_tf = []
        for region_id,cell in global_plan:
            global_plan_tf.append(self.regional_transform(self.regional_bounds_by_id(region_id),cell))
        return global_plan_tf
    

    def global_cell_value(self,global_cell):
        region_id,cell = global_cell
        return self.get_region_by_id(region_id)[1][cell]


    def verify_global_plan(self,global_plan):
        for node in global_plan:
            if self.global_cell_value(node) < 2:
                return False
        return True
    

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

