from threading import Thread
from controller.wrappers import ThreadWrapper

class Mover:

    def __init__(self,store,controller,interval=0.1):
        self._store = store
        self._tw = ThreadWrapper(self._cb,interval)
        self._controller = controller
        self._tw.start()

    def destroy(self):
        self._tw.stop()

    def _cb(self):
        if self._store.has('estimator','position'):
            plan,plan_tf = self._store.get('planner','plan')
            for point in plan_tf:
                if self._controller.move_to(point):
                    break