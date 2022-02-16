#from rich import print as rprint
#from rich.tree import Tree as rTree

from math import floor
from queue import Queue

class RangeMinimumQueryTree:
    
    class _Node:
        def __init__(self):
            self._v = None
            self._l = None
            self._r = None
    
    def __init__(self,array):
        self._high = len(array)-1
        self._tree = self._build(array)
        self._array = array

    def _build(self,array):
        
        def _build_left(i,j):
            x = self._Node()
            if i == j:
                x._v = [array[i]]
                return x
            m = floor((i+j)/2)
            x._l = _build_left(i,m)
            x._r = _build_right(m+1,j)
            b = array[j]
            x._v = [b]
            for k in range(j-1,i-1,-1):
                if array[k]<b:
                    b = array[k]
                x._v.append(b)
            x._v.reverse()
            return x

        def _build_right(i,j):
            x = self._Node()
            if i == j:
                x._v = [array[i]]
                return x
            m = floor((i+j)/2)
            x._l = _build_left(i,m)
            x._r = _build_right(m+1,j)
            b = array[i]
            x._v = [b]
            for k in range(i+1,j+1):
                if array[k]<b:
                    b = array[k]
                x._v.append(b)
            return x

        i,j = 0,len(array)-1
        x = self._Node()
        if j<0: return x
        elif j==0:
            x._v = [array[0]]
            return x
        m = floor((i+j)/2)
        x._l = _build_left(i,m)
        x._r = _build_right(m+1,j)
        return x

    def range_minimum(self,i,j):
        u,v = 0,self._high
        root = self._tree
        if i<u or j>v: return None
        if u == v: return root._v[0]
        if j-i <= 20: return min(self._array[i:j+1])
        while True:
            m = (u+v)//2
            if m < i:
                u = m+1
                root = root._r
            elif m > j:
                v = m
                root = root._l
            else:
                if root._l is None: return root._v[0]
                x,y = len(root._l._v)+i-m-1,j-m-1
                if y < 0: return root._l._v[x]
                return min(root._l._v[x],root._r._v[y])



class RangeMaxMinQueryTree:

    def __init__(self,array):
        self._high = len(array)-1
        self._tree = self._build(array)
        self._array = array

    def _build(self,array):
        
        def _build_left(i,j):
            x = [None,None,None]
            if i == j:
                x[0] = [[array[i],array[i]]]
                return x
            m = floor((i+j)/2)
            x[1] = _build_left(i,m)
            x[2] = _build_right(m+1,j)
            b = array[j]
            v = array[j]
            x[0] = [[b,v]]
            for k in range(j-1,i-1,-1):
                if array[k]<b:
                    b = array[k]
                if array[k]>v:
                    v = array[k]
                x[0].append([v,b])
            x[0].reverse()
            return x

        def _build_right(i,j):
            x = [None,None,None]
            if i == j:
                x[0] = [[array[i],array[i]]]
                return x
            m = floor((i+j)/2)
            x[1] = _build_left(i,m)
            x[2] = _build_right(m+1,j)
            b = array[i]
            v = array[i]
            x[0] = [[b,v]]
            for k in range(i+1,j+1):
                if array[k]<b:
                    b = array[k]
                if array[k]>v:
                    v = array[k]
                x[0].append([v,b])
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

    def range_maxmin(self,i,j):
        u,v = 0,self._high
        root = self._tree
        if i<u: i = u
        if j>v: j = v
        if j<i: return None
        if u == v: return root[0][0]
        if j-i <= 5: return max(self._array[i:j+1]),min(self._array[i:j+1])
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
                return max(l[x][0],r[y][0]),min(l[x][1],r[y][1])


#     def print_tree(self):
#         def p(r):
#             t = rTree(str(r._v))
#             if r._l: t.add(p(r._l))
#             if r._r: t.add(p(r._r))
#             return t
#         rprint(p(self._tree))

# def naive_range_maxmin(array,i,j):
#     return max(array[i:j+1]),min(array[i:j+1])

# from numpy.random import randint,seed
# seed(3)
# from time import time_ns

# def test():
#     F1,F2=0,0
#     num=0
#     for k in range(100):
#         a = [randint(1,100000) for _ in range(180)]
#         t1 = RangeMaxMinQueryTree(a)
#         for i in range(len(a)):
#             for j in range(i,len(a)):
#                 s = time_ns()
#                 o1 = t1.range_maxmin(i,j)
#                 e = time_ns()
#                 f1 = e-s
#                 s = time_ns()
#                 o2 = naive_range_maxmin(a,i,j)
#                 e = time_ns()
#                 f2 = e-s
#                 F1 += f1
#                 F2 += f2
#                 num += 1
#                 try:
#                     if tuple(o1) != tuple(o2):
#                         print('array:',a)
#                         print()
#                         print('subarray:',a[i:j+1])
#                         print()
#                         print('i,j:',i,'->',j)
#                         print('maxmin:',o1)
#                         print('naive_maxmin:',o2)
#                         print('FAILURE')
#                         return
#                 except Exception as e:
#                     print('Exception:',e)
#                     print('array:',a)
#                     print()
#                     print('subarray:',a[i:j+1])
#                     print()
#                     print('i,j:',i,'->',j)
#                     print('maxmin:',o1)
#                     print('naive_maxmin:',o2)
#                     print('FAILURE')
#                     return
#         V1,V2 = F1/num,F2/num
#     print('n:',len(a),f'overhead: {V1/V2:.2f}')

# test()

# class LinearQuery:
#     def __init__(self,array,key,radius):
#         self._build(array,key,radius)
    
#     def _build(self,array,key,radius):
#         Q = Queue()
#         A = []
#         n = 0
#         for _ in range(radius+1):
#             Q.put(False)
#         for i in range(radius):
#             if array[i] == key:
#                 Q.put(True)
#                 n += 1
#             else:
#                 Q.put(False)
#         for i in range(len(array)):
#             if Q.get(): n -= 1
#             if i+radius < len(array):
#                 if array[i+radius] == key:
#                     Q.put(True)
#                     n += 1
#                 else:
#                     Q.put(False)
#             if n>0:
#                 A.append(True)
#             else:
#                 A.append(False)
#         self._cache = A
    
#     def has_key(self,index):
#         return self._cache[index]

# class BlockQuery:
#     def __init__(self,block):
#         pass

# l = LinearQuery([1,0,1,2,3,4,0,1,2,3,4,5,6],0,3)
# for i in range(len(l._cache)):
#     print(i,':',l.has_key(i))