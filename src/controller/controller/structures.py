from math import ceil,atan2,pi,atan,floor,cos,sqrt,isinf
from sensor_msgs.msg import LaserScan
import numpy as np
from controller.auxiliary import normalize_angle,merge_bounding_boxes,constrain


class BitField:
    
    def __init__(self, bit_field_size_in_bits, word_size_in_bits = 32):
        self.__bit_field__ = [0]*ceil(bit_field_size_in_bits/word_size_in_bits)
        self.__word_size__ = word_size_in_bits
        self.__length__ = bit_field_size_in_bits
    
    def bit_modify(self, index, value):
        i = index//self.__word_size__
        j = index%self.__word_size__
        self.__bit_field__[i] ^= (((self.__bit_field__[i]>>j)&1)^value)<<j
    
    def bit_set(self, index):
        self.__bit_field__[index//self.__word_size__] |= 1<<(index%self.__word_size__)

    def bit_unset(self, index):
        i = index//self.__word_size__
        j = index%self.__word_size__
        self.__bit_field__[i] ^= ((self.__bit_field__[i]>>j)&1)<<j
    
    def bit_get(self, index):
        return (self.__bit_field__[index//self.__word_size__]>>(index%self.__word_size__))&1
    
    def for_each(self, callback):
        for index in range(self.__length__):
            callback(index,self.bit_get(index))


class Patch:
    
    def __init__(self, msg: LaserScan, position):
        self.__obstacle_map__ = BitField(len(msg.ranges))
        self.__obstacles__ = []
        self.__origin__ = position
        for i,d in enumerate(msg.ranges):
            if not isinf(d):
                self.__obstacle_map__.bit_set(i)
                self.__obstacles__.append(d)

    def for_each_obstacle(self,callback):
        i = 0
        def callback2(t,b):
            nonlocal i
            if b:
                callback(self.__obstacles__[i],t)
                i += 1
        self.__obstacle_map__.for_each(callback2)


    def rasterize(self, sensor_radius, resolution, obstacle_lookup_radius):
        
        cx,cy,ct = self.__origin__
        xmin,ymax = cx-sensor_radius,cy+sensor_radius
        
        row = round(2*sensor_radius/resolution)
        col = row
        
        obstacle_map = [None]*181
        j=0
        for i in range(181):
            if self.__obstacle_map__.bit_get(i):
                obstacle_map[i] = self.__obstacles__[j]
                j += 1

        rasterized_patch = np.full(shape=(row,col),dtype=np.uint8,fill_value=2)

        for i in range(row):
            for j in range(col):
                
                x,y = xmin-cx+resolution/2+j*resolution,ymax-cy-resolution/2-i*resolution
                r,t = sqrt(x*x+y*y),normalize_angle(atan2(y,x)-ct+pi/2)

                if t < 0 or r > sensor_radius: pass
                elif r == 0:
                    rasterized_patch[i,j] = 0
                else:
                    if obstacle_lookup_radius > r:
                        tmin,tmax = 0,pi
                    else:
                        dt = atan(obstacle_lookup_radius/r)
                        if t < pi/2:
                            tmin,tmax = max(t-dt,0),t+dt
                        else:
                            tmin,tmax = t-dt,min(t+dt,pi)
                    begin,end = ceil(tmin*180/pi),floor(tmax*180/pi)
                    for t1 in range(begin,end+1):
                        r1 = obstacle_map[t1]
                        if r1 is not None:
                            d = r*cos(t-t1*pi/180)
                            if r*r+r1*r1-obstacle_lookup_radius*obstacle_lookup_radius <= 2*r1*d:
                                rasterized_patch[i,j] = 1
                                break
                            if d < r1:
                                rasterized_patch[i,j] = 0
                        else:
                            rasterized_patch[i,j] = 0
        
        return (xmin,cy-sensor_radius,cx+sensor_radius,ymax),rasterized_patch


class Map:
    
    def __init__(self,sensor_radius,resolution,obstacle_lookup_radius):
        self.__map__ = None
        self.__sensor_radius__ = sensor_radius
        self.__resolution__ = resolution
        self.__obstacle_lookup_radius__ = obstacle_lookup_radius
    
    def __transform__(self,i,j,xmin,ymax):
        return xmin+(j+0.5)*self.__resolution__,ymax-(i+0.5)*self.__resolution__
    
    def __inverse_transform__(self,x,y,xmin,ymax):
        return floor((ymax-y)/self.__resolution__),floor((x-xmin)/self.__resolution__)
    
    def merge_patch(self, patch: Patch):

        if self.__map__ is None:
            self.__bounds__,self.__map__ = patch.rasterize(self.__sensor_radius__,self.__resolution__,self.__obstacle_lookup_radius__)
            return

        patch_bounds,rasterized_patch = patch.rasterize(self.__sensor_radius__,self.__resolution__,self.__obstacle_lookup_radius__)
        merged_bounds = merge_bounding_boxes(self.__bounds__,patch_bounds)
        merged_num_rows,merged_num_cols = floor((merged_bounds[3]-merged_bounds[1])/self.__resolution__),floor((merged_bounds[2]-merged_bounds[0])/self.__resolution__)
        merged_map = np.full(shape=(merged_num_rows,merged_num_cols),dtype=np.uint8,fill_value=2)

        for i in range(self.__map__.shape[0]):
            for j in range(self.__map__.shape[1]):
                if self.__map__[i,j] != 2:
                    t1 = self.__transform__(i,j,self.__bounds__[0],self.__bounds__[3])
                    t2 = self.__inverse_transform__(t1[0],t1[1],merged_bounds[0],merged_bounds[3])
                    k = constrain(t2[0],0,merged_num_rows-1)
                    l = constrain(t2[1],0,merged_num_cols-1)
                    merged_map[k,l] = self.__map__[i,j]

        for i in range(rasterized_patch.shape[0]):
            for j in range(rasterized_patch.shape[1]):
                if rasterized_patch[i,j] != 2:
                    t1 = self.__transform__(i,j,patch_bounds[0],patch_bounds[3])
                    t2 = self.__inverse_transform__(t1[0],t1[1],merged_bounds[0],merged_bounds[3])
                    k = constrain(t2[0],0,merged_num_rows-1)
                    l = constrain(t2[1],0,merged_num_cols-1)
                    merged_map[k,l] = rasterized_patch[i,j]

        self.__bounds__= merged_bounds
        self.__map__ = merged_map
    
    def get_map(self): return self.__map__


class CriticalMap:
    
    def __init__(self,map: Map,critical_radius):
        self.__map__ = map
        self.__critical_radius__ = critical_radius
        self.__critical_map__ = None
        self.update_critical_map()

    def update_critical_map(self):
        matrix = self.__map__.get_map()
        if matrix is None: return
        rows,cols = matrix.shape
        self.__bounds__ = self.__map__.__bounds__[0]-self.__critical_radius__,self.__map__.__bounds__[1]-self.__critical_radius__,self.__map__.__bounds__[2]+self.__critical_radius__,self.__map__.__bounds__[3]+self.__critical_radius__
        self.__rows__,self.__columns__ = ceil((self.__bounds__[3]-self.__bounds__[1])/self.__map__.__resolution__),ceil((self.__bounds__[2]-self.__bounds__[0])/self.__map__.__resolution__)
        critical_map = np.full((self.__rows__,self.__columns__),2,np.uint8)
        delta = ceil(self.__critical_radius__/self.__map__.__resolution__)
        extention = ceil(2*self.__critical_radius__/self.__map__.__resolution__)
        for i in range(rows):
            for j in range(cols):
                if matrix[i,j] == 1:
                    for k in range(i,1+min(i+extention,self.__rows__-1)):
                        for l in range(j,1+min(j+extention,self.__columns__-1)):
                            critical_map[k,l] = 1
                elif matrix[i,j] == 0:
                    critical_map[min(i+delta,self.__rows__-1),min(self.__columns__-1,j+delta)] = 0
        self.__critical_map__ = critical_map
    
    def get_critical_map(self):
        return self.__critical_map__

    def tf(self,i,j):
        xmin = self.__bounds__[0]
        ymax = self.__bounds__[3]
        return xmin+(j+0.5)*self.__map__.__resolution__,ymax-(i+0.5)*self.__map__.__resolution__
    
    def itf(self,x,y):
        xmin = self.__bounds__[0]
        ymax = self.__bounds__[3]
        return floor((ymax-y)/self.__map__.__resolution__), floor((x-xmin)/self.__map__.__resolution__)


class Curve:

    def __init__(self,resolution=0.000001):
        self._resolution = resolution
        
    def radius_of_curvature(self,x):
        d1 = abs(self.derivative(x,1))
        d2 = abs(self.derivative(x,2))
        return (1+d1*d1)**1.5/d2

    def derivative(self,x,order):
        if order < 0: return
        cache = np.zeros((order+1,order+1),np.float64)
        for j in range(order+1):
            cache[0,j] = self.at(x+j*self._resolution)
        for i in range(1,order+1):
            for j in range(order+1-i):
                cache[i,j] = (cache[i-1,j+1]-cache[i-1,j])/self._resolution
        return cache[order,0]


class Polynomial(Curve):
    
    def __init__(self,data_points,high_precision=False):
        Curve.__init__(self)
        degree_plus_1 = len(data_points)
        X = np.zeros((degree_plus_1,degree_plus_1),np.float64)
        Y = np.zeros((degree_plus_1,1),np.float64)
        for i,point in enumerate(data_points):
            Y[i,0] = point[1]
            element = 1
            for j in range(degree_plus_1):
                X[i,j] = element
                element *= point[0]
        self._c = np.reshape(np.matmul(np.linalg.inv(X),Y),(degree_plus_1))
        self._hp = False
        if high_precision:
            self._s = np.ones((degree_plus_1),np.byte)
            for i in range(1,degree_plus_1):
                if self._c[i] >= 0:
                    self._c[i] **= 1/i
                else:
                    self._c[i] = (-self._c[i])**1/i
                    self._s[i] = -1
            self._hp = True
    
    def at(self,point):
        if not self._hp:
            y,p=0,1
            for c in self._c:
                y += p*c
                p *= point
        else:
            y=self._c[0]
            for i in range(1,len(self._c)):
                y += self._s[i]*(self._c[i]*point)**i
        return y

