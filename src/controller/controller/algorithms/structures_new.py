from random import random
from math import ceil,atan2,pi,atan,floor,cos,sqrt,isinf,asin
from time import time_ns
import numpy as np
# from controller.auxiliary import merge_bounding_boxes
import cv2

def merge_bounding_boxes(bb1,bb2):
    return min(bb1[0],bb2[0]),min(bb1[1],bb2[1]),max(bb1[2],bb2[2]),max(bb1[3],bb2[3])

def normalize_angle(angle):
    two_pi = 2*pi
    angle %= two_pi
    if angle < -pi:
        return angle+two_pi
    elif angle > pi:
        return angle-two_pi
    return angle


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
    
    def __init__(self):
        pass

    def for_each_obstacle(self,callback):
        i = 0
        def callback2(t,b):
            nonlocal i
            if b:
                callback(self.__obstacles__[i],t)
                i += 1
        self.__obstacle_map__.for_each(callback2)

    def copy(self):
        p = Patch()
        p.__obstacle_map__ = BitField(self.__obstacle_map__.__length__)
        p.__obstacles__ = []
        p.__origin__ = self.__origin__
        j = 0
        for i in range(self.__obstacle_map__.__length__):
            if self.__obstacle_map__.bit_get(i):
                p.__obstacle_map__.bit_set(i)
                p.__obstacles__.append(self.__obstacles__[j])
                j += 1
        return p
    
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

        rasterized_patch = np.full(shape=(row,col),dtype=np.uint8,fill_value=4)

        for i in range(row):
            for j in range(col):
                
                x,y = xmin-cx+resolution/2+j*resolution,ymax-cy-resolution/2-i*resolution
                r,t = sqrt(x*x+y*y),normalize_angle(atan2(y,x)-ct+pi/2)

                if t < 0 or r > sensor_radius: pass
                elif r == 0:
                    rasterized_patch[i,j] = 3
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
                                rasterized_patch[i,j] = 0
                                break
                            if d < r1:
                                rasterized_patch[i,j] = 3
                        else:
                            rasterized_patch[i,j] = 3
        
        return (xmin,cy-sensor_radius,cx+sensor_radius,ymax),rasterized_patch


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


def rasterize_patch(patch,sensor_radius,resolution,obstacle_radius):

    cx,cy,ct = patch.__origin__

    rpr,rpc = ceil(sqrt(8)*sensor_radius/resolution),ceil(sqrt(8)*sensor_radius/resolution)
    rb = (cx-sqrt(2)*sensor_radius,cy-sqrt(2)*sensor_radius,cx+sqrt(2)*sensor_radius,cy+sqrt(2)*sensor_radius)
    rp = np.full((rpr,rpc),4,np.uint8)

    pom,po = patch.__obstacle_map__,patch.__obstacles__
    j=0
    poa = []
    for i in range(181):
        if pom.bit_get(i):
            poa.append(po[j])
            j += 1
        else:
            poa.append(2**31)

    pob_ds = RangeMinimumTree(poa)

    for i in range(rpr):
        for j in range(rpc):

            x = rb[0]+(j+0.5)*resolution
            y = rb[3]-(i+0.5)*resolution
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
                    if s > r+obstacle_radius:
                        rp[i,j] = 3
                    elif s >= r-obstacle_radius:
                        rp[i,j] = 0
    return rp

def gen_patch(sensor_radius,obstacle_probability,min_dist=0.1):
    p = Patch()
    p.__origin__ = (random()*200-100,random()*200-100,random()*2*pi-pi)
    om = BitField(181)
    o = []
    for i in range(181):
        if random() < obstacle_probability:
            om.bit_set(i)
            o.append(min_dist+random()*(sensor_radius-min_dist))
    p.__obstacle_map__ = om
    p.__obstacles__ = o
    return p



import cv2

def test_speed():
    
    for resolution in [0.5]:
        for ob_rad in [0.5]:
            t1,t2,n = 0,0,0
            for obstacle_prob in [0.01]:
                p = gen_patch(9.0,obstacle_prob)
                for i in range(100):
                    s = time_ns()
                    rasterize_patch(p,9.0,resolution,ob_rad)
                    e = time_ns()
                    t1 += e-s
                    s = time_ns()
                    p.rasterize(9.0,resolution,ob_rad)
                    e = time_ns()
                    t2 += e-s
                    n += 1
                    # return
                    # y1 = visualize_patch(p)
                    # y2 = visualize_patch_2(p)
                    # cv2.imshow('Original',y2)
                    # cv2.imshow('New',y1)
                    # while cv2.waitKey(100) <= 0: pass
            f = t1/t2
            print('resolution:',resolution,'ob_radius:',ob_rad,'overhead:',f)

def visualize_patch_2(patch):
    _,rp = patch.rasterize(9.0,0.5,0.5)
    r,c = rp.shape
    x = np.full((r,c,3),128,np.uint8)
    for i in range(r):
        for j in range(c):
            if rp[i,j] == 3:
                x[i,j] = [255,255,255]
            elif rp[i,j] == 0:
                x[i,j] = [0,0,0]
    return cv2.resize(x,(500,500))

def visualize_patch(patch):
    rp = rasterize_patch(patch,9.0,0.5,0.5)
    r,c = rp.shape
    x = np.full((r,c,3),128,np.uint8)
    for i in range(r):
        for j in range(c):
            if rp[i,j] == 3:
                x[i,j] = [255,255,255]
            elif rp[i,j] == 0:
                x[i,j] = [0,0,0]
    return cv2.resize(x,(500,500))
    cv2.imshow('Rasterized Patch',x)
    cv2.waitKey(100)

#test_speed()

#padding >= hyper_critcial_radius >= critical_radius
def construct_cmap_from_patch(cmap, cmap_bounds, patch, sensor_radius, resolution, padding, obstacle_radius):

    cx,cy,ct = patch.__origin__
    
    pb = cx-sensor_radius,cy-sensor_radius,cx+sensor_radius,cy+sensor_radius

    if cmap is not None:
        cb = merge_bounding_boxes(cmap_bounds,pb)
    else:
        cb = pb
    
    cb = cb[0]-padding,cb[1]-padding,cb[2]+padding,cb[3]+padding
    
    cr,cc = ceil((cb[2]-cb[0])/resolution),ceil((cb[3]-cb[1])/resolution)
    cm = np.full((cr,cc),4,np.uint8)
    _cr,_cc = cmap.shape

    tf = lambda p,b,r: (b[0]+(p[1]+0.5)*r,b[1]+(p[0]+0.5)*r)
    itf = lambda p,b,r: (floor((p[1]-b[1])/r),floor((p[0]-b[0])/r))

    if cmap is not None:
        for i in range(_cr):
            for j in range(_cc):
                cm[itf(tf((i,j),cmap_bounds,resolution),cb,resolution)] = cmap[i,j]
    
    rpr,rpc = ceil(sqrt(8)*sensor_radius/resolution),ceil(sqrt(8)*sensor_radius/resolution)
    rb = (cx-sqrt(2)*sensor_radius,cy-sqrt(2)*sensor_radius,cx+sqrt(2)*sensor_radius,cy+sqrt(2)*sensor_radius)
    rp = np.full((rpr,rpc),4,np.uint8)

    pom,po = patch.__obstacle_map__,patch.__obstacles__
    j=0
    poa = []
    for i in range(181):
        if pom.bit_get(i):
            poa.append(po[j])
            j += 1
        else:
            poa.append(2**31)

    pob_ds = RangeMinimumTree(poa)

    for i in range(rpr):
        for j in range(rpc):

            x = rb[0]+(j+0.5)*resolution
            y = rb[3]-(i+0.5)*resolution
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
                    p = itf((x,y),cb,resolution)
                    if s > r+obstacle_radius:
                        cm[p] = 3
                    elif s >= r-obstacle_radius:
                        cm[p] = 0
    return rp