from threading import Thread
from controller.wrappers import ThreadWrapper
from controller.auxiliary import cartesian_dist

class Mover:

    def __init__(self,store,controller):
        self._store = store
        self._tw = ThreadWrapper(self._cb,store.get('general','mover_node_callback_interval'))
        self._tw.__thread__.setName("mover-thread")
        self._controller = controller
        self._tw.start()

    def destroy(self):
        self._tw.stop()

    def _find_index_in_plan(self,plan_tf,position):
        point = position[0],position[1]
        min_i,min_d = None,None
        for i in range(len(plan_tf)):
            d = cartesian_dist(plan_tf[i],point)
            if min_i is None or d <= min_d:
                min_i = i
                min_d = d
        
        if min_i == 0: return 1
        if min_i == len(plan_tf)-1: min_i
        
        dx = (plan_tf[min_i][0]-plan_tf[min_i-1][0])*(plan_tf[min_i][0]-point[0])
        dy = (plan_tf[min_i][1]-plan_tf[min_i-1][1])*(plan_tf[min_i][1]-point[1])
        
        if dx+dy >= 0: return min_i 
        return min_i+1

    def _cb(self):
        if self._store.has('general','shutdown'):
            if not self._store.get('general','executor')._is_shutdown:
                self._store.get('general','executor').shutdown()
            return
        if self._store.get('critical','interrupt'):
            return
        if self._store.has('estimator','position'):
            plan,plan_tf = self._store.get('planner','plan')
            if not plan or len(plan) < 2: return
            index = self._find_index_in_plan(plan_tf,self._store.get('estimator','position'))
            for point in plan_tf[index:]:
                if self._controller.move_to(point):
                    break                
