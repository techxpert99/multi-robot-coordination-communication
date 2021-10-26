from controller.wrappers import NodeWrapper
from sensor_msgs.msg import LaserScan
from controller.structures import Patch


class SensorNode:
    
    def __init__(self,store):
        self.__wrapper__ = NodeWrapper(store.get('names','sensor'),store)
        self.__wrapper__.create_subscription('laser.in',self.__laser_callback__)
        self._store = store
    
    def __laser_callback__(self, msg: LaserScan):
        if self._store.has('estimator','position'):
            self._store.set('sensor','patch',Patch(msg,self._store.get('estimator','position')))

    def destroy(self):
        self.__wrapper__.destroy()
