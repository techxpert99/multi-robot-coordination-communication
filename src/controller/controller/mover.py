from math import sqrt
from threading import Thread
from controller.wrappers import ThreadWrapper
from controller.auxiliary import cartesian_dist

class Mover:

    def __init__(self,store,controller):
        self._store = store
        self._tw = ThreadWrapper(self._cb,store.get('general','mover_node_callback_interval'))
        self._tw.__thread__.setName(store.get('names','namespace')+'/mover')
        self._controller = controller
        self._tw.start()

    def destroy(self):
        self._tw.stop()

    def _process_plan(self,plan_tf,position):
        
        c0,c1,_ = position

        cache = None

        for i in range(1,len(plan_tf)):
            
            a0,a1 = plan_tf[i-1]
            b0,b1 = plan_tf[i]
            
            da = a0-b0
            db = a1-b1
            da2 = da*da
            db2 = db*db
            dab = da*db
            da2db2 = da2+db2

            d0 = (da2*c0+db2*a0+dab*(c1-a1))/da2db2
            d1 = (db2*c1+dab*(c0-a0)+da2*a1)/da2db2

            dx = d0-c0
            dy = d1-c1

            distance = sqrt(dx*dx+dy*dy)

            if cache is None or distance < cache[0]:
                cache = (distance,(d0,d1),i)

        return cache[1],cache[2]

    # def _find_index_in_plan(self,plan_tf,position):
    #     point = position[0],position[1]
    #     min_i,min_d = None,None
    #     for i in range(len(plan_tf)):
    #         d = cartesian_dist(plan_tf[i],point)
    #         if min_i is None or d <= min_d:
    #             min_i = i
    #             min_d = d
        
    #     if min_i == 0: return 1
    #     if min_i == len(plan_tf)-1: min_i
        
    #     dx = (plan_tf[min_i][0]-plan_tf[min_i-1][0])*(plan_tf[min_i][0]-point[0])
    #     dy = (plan_tf[min_i][1]-plan_tf[min_i-1][1])*(plan_tf[min_i][1]-point[1])
        
    #     if dx+dy >= 0: return min_i 
    #     return min_i+1

    def _cb(self):
        if self._store.get('critical','interrupt'):
            return
        if self._store.has('estimator','position'):
            plan_tf = self._store.get('planner','plan_tf')
            if not plan_tf or len(plan_tf) < 2: return
            initial_point,index = self._process_plan(plan_tf,self._store.get('estimator','position'))
            if self._controller.move_to(initial_point):
                return
            for i in range(index,len(plan_tf)):
                if self._controller.move_to(plan_tf[i]):
                    return
            self._store.set('planner','goal',None)