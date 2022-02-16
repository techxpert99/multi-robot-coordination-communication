from math import ceil,atan2, cos,pi,floor, sin,sqrt,asin
import numpy as np
from numpy.core.fromnumeric import reshape
from controller.auxiliary import merge_bounding_boxes

class RangeMinimumTree:

    def __init__(self,array):
        self._high = len(array)-1
        self._tree = self._build(array)
        self._array = array

    def _build(self,array):
        
        def _build_left(i,j):
            x = [None,None,None]
            if i == j:
                x[0] = [array[i]]
                return x
            m = floor((i+j)/2)
            x[1] = _build_left(i,m)
            x[2] = _build_right(m+1,j)
            b = array[j]
            x[0] = [b]
            for k in range(j-1,i-1,-1):
                if array[k]<b:
                    b = array[k]
                x[0].append(b)
            x[0].reverse()
            return x

        def _build_right(i,j):
            x = [None,None,None]
            if i == j:
                x[0] = [array[i]]
                return x
            m = floor((i+j)/2)
            x[1] = _build_left(i,m)
            x[2] = _build_right(m+1,j)
            b = array[i]
            x[0] = [b]
            for k in range(i+1,j+1):
                if array[k]<b:
                    b = array[k]
                x[0].append(b)
            return x

        i,j = 0,len(array)-1
        x = [None,None,None]
        if j<0: return x
        elif j==0:
            x[0] = [array[0]]
            return x
        m = floor((i+j)/2)
        x[1] = _build_left(i,m)
        x[2] = _build_right(m+1,j)
        return x

    def range_minimum(self,i,j):
        u,v = 0,self._high
        root = self._tree
        if i<u: i = u
        if j>v: j = v
        if j<i: return None
        if u == v: return root[0][0]
        if j-i <= 5: return min(self._array[i:j+1])
        while True:
            m = (u+v)//2
            if m < i:
                u = m+1
                root = root[2]
            elif m > j:
                v = m
                root = root[1]
            else:
                if root[1] is None: return root[0][0]
                l,r = root[1][0],root[2][0]
                x,y = len(l)+i-m-1,j-m-1
                if y < 0: return l[x]
                return min(l[x],r[y])

class OptimizedCriticalMap:
    
    def __init__(self,sensor_radius,resolution,padding,obstacle_radius,critical_radius,hypercritical_radius):
         self._params = sensor_radius,resolution,padding,obstacle_radius,critical_radius,hypercritical_radius
         self._cmap_tuple = None

    def merge_patch(self,patch):
        if patch is None: return
        self._cmap_tuple = self._construct_cmap_from_patch(self._cmap_tuple,patch,self._params[0],self._params[1],self._params[2],self._params[3],self._params[4],self._params[5])   

    def merge_patch_new(self,patch):
        if patch is None: return
        self._cmap_tuple = self._build_cmap_from_patch(self._cmap_tuple,patch,self._params[0],self._params[1],self._params[2],self._params[3],self._params[4],self._params[5])   


    def get_critical_map(self):
        cm = self._cmap_tuple
        if cm is not None:
            return self._cmap_tuple[0]
        return None
    
    def itf(self,x,y):
        b = self._cmap_tuple[1]
        r = self._params[1]
        return floor((b[3]-y)/r),floor((x-b[0])/r)
    
    def tf(self,i,j):
        b = self._cmap_tuple[1]
        r = self._params[1]
        return b[0]+(j+0.5)*r, b[3]-(i+0.5)*r

    #padding >= hyper_critcial_radius >= critical_radius
    def _construct_cmap_from_patch(self, cmap_tuple, patch, sensor_radius, resolution, padding, obstacle_radius,critical_radius,hypercritical_radius):

        if cmap_tuple is not None:
            cmap,cmap_bounds = cmap_tuple
        
        cx,cy,ct = patch.__origin__
        
        offset = sensor_radius+2*hypercritical_radius+padding
        pb = cx-sensor_radius,cy-sensor_radius,cx+sensor_radius,cy+sensor_radius
        eb = cx-offset,cy-offset,cx+offset,cy+offset

        if cmap_tuple is not None:
            cb = merge_bounding_boxes(cmap_bounds,eb)
        else:
            cb = eb
        
        cr,cc = ceil((cb[3]-cb[1])/resolution),ceil((cb[2]-cb[0])/resolution)
        pr,pc = ceil((pb[3]-pb[1])/resolution),ceil((pb[2]-pb[0])/resolution)

        cm = np.full((cr,cc),4,np.uint8)

        if cmap_tuple is not None:
            _cr,_cc = cmap.shape
            for i in range(_cr):
                for j in range(_cc):
                    x,y = cmap_bounds[0]+(j+0.5)*resolution, cmap_bounds[3]-(i+0.5)*resolution
                    p = floor((cb[3]-y)/resolution),floor((x-cb[0])/resolution)
                    if p[0] >= 0 and p[0] < cr and p[1] >= 0 and p[1] < cc:
                        cm[p] = cmap[i,j]

        pom,po = patch.__obstacle_map__,patch.__obstacles__
        j=0
        poa = []
        for i in range(pom.__length__):
            if pom.bit_get(i):
                poa.append(po[j])
                j += 1
            else:
                poa.append(2**31)

        pob_ds = RangeMinimumTree(poa)

        for i in range(pr):
            for j in range(pc):

                x = pb[0]+(j+0.5)*resolution
                y = pb[3]-(i+0.5)*resolution
                r = sqrt((x-cx)*(x-cx)+(y-cy)*(y-cy))
                t = atan2(y-cy,x-cx)+pi/2-ct

                if t < -pi: t += 2*pi
                if t > pi: t -= 2*pi

                if r <= sensor_radius and t >= 0 and t <= pi:
                    if r < obstacle_radius:
                        s = pob_ds.range_minimum(0,181)
                    else:
                        dt_d = ceil(asin(obstacle_radius/r)*180/pi)
                        t_d = floor(180*t/pi)
                        s = pob_ds.range_minimum(t_d-dt_d,t_d+dt_d)
                    if s is not None:
                        p = floor((cb[3]-y)/resolution),floor((x-cb[0])/resolution)
                        if p[0] >= 0 or p[0] < cr or p[1] >= 0 or p[1] < cc:
                            if s > r+obstacle_radius:
                                cm[p] = 3
                            elif s >= r-obstacle_radius:
                                cm[p] = 0
        
        offset2 = sensor_radius+hypercritical_radius
        
        i2,i3 = floor((cb[3]-cy-offset2)/resolution),floor((cb[3]-cy+offset2)/resolution)
        j2,j3 = floor((cx-offset2-cb[0])/resolution),floor((cx+offset2-cb[0])/resolution)
        
        hrad,crad = ceil(hypercritical_radius/resolution),ceil(critical_radius/resolution)

        def build_cache_region(i,c1,c2,c3,c4):
            
            c3 += 1
            c4 += 1

            for j in range(j2-hrad,j2-crad):
                if cm[i,j] == 0:
                    c8 = j+hrad+1
                    for k in range(c1,c4):
                        cache[k-i2][j2-j2][1] += 1
                        cache[k-i2][c8-j2][1] -= 1
            
            for j in range(j2-crad,j2+1):
                if cm[i,j] == 0:
                    c7,c8 = j+crad+1,j+hrad+1
                    for k in range(c2,c3):
                        cache[k-i2][j2-j2][0] += 1
                        cache[k-i2][c7-j2][0] -= 1
                    for k in range(c1,c4):
                        cache[k-i2][j2-j2][1] += 1
                        cache[k-i2][c8-j2][1] -= 1
            
            for j in range(j3+1,j3+crad+1):
                if cm[i,j] == 0:
                    c5,c6 = j-hrad,j-crad
                    for k in range(c2,c3):
                        cache[k-i2][c6-j2][0] += 1
                    for k in range(c1,c4):
                        cache[k-i2][c6-j2][1] += 1

            for j in range(j3+crad+1,j3+hrad+1):
                if cm[i,j] == 0:
                    c5 = j-hrad
                    for k in range(c1,c4):
                        cache[k-i2][c5-j2][1] += 1

            for j in range(j2,j2+crad+1):
                if cm[i,j] == 0:
                    c7,c8 = j+crad+1,j+hrad+1
                    for k in range(c2,c3):
                        cache[k-i2][j2-j2][0] += 1
                        cache[k-i2][c7-j2][0] -= 1
                    for k in range(c1,c4+1):
                        cache[k-i2][j2-j2][1] += 1
                        cache[k-i2][c8-j2][1] -= 1
            
            for j in range(j2+crad+1,j2+hrad+1):
                if cm[i,j] == 0:
                    c6,c7,c8 = j-crad,j+crad+1,j+hrad+1
                    for k in range(c2,c3):
                        cache[k-i2][c6-j2][0] += 1
                        cache[k-i2][c7-j2][0] -= 1
                    for k in range(c1,c4):
                        cache[k-i2][j2-j2][1] += 1
                        cache[k-i2][c8-j2][1] -= 1

            for j in range(j2+hrad+1,j3-hrad):
                if cm[i,j] == 0:
                    c5,c6,c7,c8 = j-hrad,j-crad,j+crad+1,j+hrad+1
                    for k in range(c2,c3):
                        cache[k-i2][c6-j2][0] += 1
                        cache[k-i2][c7-j2][0] -= 1
                    for k in range(c1,c4):
                        cache[k-i2][c5-j2][1] += 1
                        cache[k-i2][c8-j2][1] -= 1
            
            for j in range(j3-hrad,j3-crad):
                if cm[i,j] == 0:
                    c5,c6,c7 = j-hrad,j-crad,j+crad+1
                    for k in range(c2,c3):
                        cache[k-i2][c6-j2][0] += 1
                        cache[k-i2][c7-j2][0] -= 1
                    for k in range(c1,c4):
                        cache[k-i2][c5-j2][1] += 1
            
            for j in range(j3-crad,j3+1):
                if cm[i,j] == 0:
                    c5,c6 = j-hrad,j-crad
                    for k in range(c2,c3):
                        cache[k-i2][c6-j2][0] += 1
                    for k in range(c1,c4):
                        cache[k-i2][c5-j2][1] += 1
        
        def build_cache_hypercritical_region(i,c1,c4):

            c1 += 1
            c4 += 1

            for j in range(j2-hrad,j2):
                if cm[i,j] == 0:
                    c8 = j+hrad+1
                    for k in range(c1,c4):
                        cache[k-i2][j2-j2][1] += 1
                        cache[k-i2][c8-j2][1] -= 1
            for j in range(j3+1,j3+hrad+1):
                if cm[i,j] == 0:
                    c5 = j-hrad
                    for k in range(c1,c4):
                        cache[k-i2][c5-j2][1] += 1

            for j in range(j2,j2+crad+1):
                if cm[i,j] == 0:
                    c8 = j+hrad+1
                    for k in range(c1,c4+1):
                        cache[k-i2][j2-j2][1] += 1
                        cache[k-i2][c8-j2][1] -= 1
            
            for j in range(j2+crad+1,j2+hrad+1):
                if cm[i,j] == 0:
                    c8 = j+hrad+1
                    for k in range(c1,c4):
                        cache[k-i2][j2-j2][1] += 1
                        cache[k-i2][c8-j2][1] -= 1

            for j in range(j2+hrad+1,j3-hrad):
                if cm[i,j] == 0:
                    c5,c8 = j-hrad,j+hrad+1
                    for k in range(c1,c4):
                        cache[k-i2][c5-j2][1] += 1
                        cache[k-i2][c8-j2][1] -= 1
            
            for j in range(j3-hrad,j3-crad):
                if cm[i,j] == 0:
                    c5 = j-hrad
                    for k in range(c1,c4):
                        cache[k-i2][c5-j2][1] += 1
            
            for j in range(j3-crad,j3+1):
                if cm[i,j] == 0:
                    c5 = j-hrad
                    for k in range(c1,c4):
                        cache[k-i2][c5-j2][1] += 1

        def build_cache():
            for i in range(i2-hrad,i2-crad):
                build_cache_hypercritical_region(i,i2,i+hrad)
            for i in range(i2-crad,i2):
                build_cache_region(i,i2,i2,i+crad,i+hrad)
            for i in range(i2+crad+1,i2+hrad+1):
                build_cache_region(i,i2,i-crad,i+crad,i+hrad)
            for i in range(i2+hrad+1,i3-hrad):
                build_cache_region(i,i-hrad,i-crad,i+crad,i+hrad)
            for i in range(i3-hrad,i3-crad):
                build_cache_region(i,i-hrad,i-crad,i+crad,i3)
            for i in range(i3-crad,i3+1):
                build_cache_region(i,i-hrad,i-crad,i3,i3)
            for i in range(i3+1,i3+crad+1):
                build_cache_region(i,i-hrad,i-crad,i3,i3)
            for i in range(i3+crad+1,i3+hrad+1):
                build_cache_hypercritical_region(i,i-hrad,i3)
        
        cache = [[[0,0] for j in range(j3-j2+1)] for i in range(i3-i2+1)]
        build_cache()

        for i in range(i2,i3+1):
            nc,nh = 0,0
            for j in range(j2,j3+1):
                nc += cache[i-i2][j-j2][0]
                nh += cache[i-i2][j-j2][1]
                if cm[i,j] != 0:
                    if nc>0:
                        cm[i,j] = 1
                    elif nh>0:
                        cm[i,j] = 2
        
        return (cm,cb)

    # padding > 2*hyper_critical_radius
    # hyper_critical_radius > critical_radius
    def _build_cmap_from_patch(self, cmap_tuple, patch, sensor_radius, resolution, padding, obstacle_radius,critical_radius,hypercritical_radius):

        if cmap_tuple is not None:
            cmap,cmap_bounds = cmap_tuple
        
        cx,cy,ct = patch.__origin__
        
        offset = sensor_radius+2*hypercritical_radius+padding
        pb = cx-sensor_radius,cy-sensor_radius,cx+sensor_radius,cy+sensor_radius
        eb = cx-offset,cy-offset,cx+offset,cy+offset

        if cmap_tuple is not None:
            cb = merge_bounding_boxes(cmap_bounds,eb)
        else:
            cb = eb
        
        cr,cc = ceil((cb[3]-cb[1])/resolution),ceil((cb[2]-cb[0])/resolution)
        pr,pc = ceil((pb[3]-pb[1])/resolution),ceil((pb[2]-pb[0])/resolution)

        cm = np.full((cr,cc),4,np.uint8)

        if cmap_tuple is not None:
            _cr,_cc = cmap.shape
            for i in range(_cr):
                for j in range(_cc):
                    x,y = cmap_bounds[0]+(j+0.5)*resolution, cmap_bounds[3]-(i+0.5)*resolution
                    p = floor((cb[3]-y)/resolution),floor((x-cb[0])/resolution)
                    if p[0] >= 0 and p[0] < cr and p[1] >= 0 and p[1] < cc:
                        cm[p] = cmap[i,j]

        pom,po = patch.__obstacle_map__,patch.__obstacles__
        j=0
        poa = []
        for i in range(pom.__length__):
            if pom.bit_get(i) == 1 and po[j] <= sensor_radius:
                poa.append(po[j])
                j += 1
            else:
                poa.append(None)

        u0,v0 = floor((cb[3]-(pb[3]-0.5*resolution))/resolution),floor((pb[0]-cb[0])/resolution)
        y = pb[3]+resolution/2

        for i in range(pr):
            y -= resolution
            x = pb[0]-resolution/2
            for j in range(pc):
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

        return (cm,cb)


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