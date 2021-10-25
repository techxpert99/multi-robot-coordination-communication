from controller.wrappers import ThreadWrapper
from controller.structures import Map,CriticalMap


class Mapper:

    def __init__(self,store,merge_interval=1,sensor_radius=9.0,resolution=0.3,obstacle_lookup_radius=0.3,critical_radius=0.3):
        self.__threadw__ = ThreadWrapper(self.__callback__,merge_interval)
        map = Map(sensor_radius,resolution,obstacle_lookup_radius)
        cmap = CriticalMap(map,critical_radius)
        self._store = store
        self._store.set('mapper','map',map)
        self._store.set('mapper','cmap',cmap)
        self.__threadw__.start()
    
    def __callback__(self):
        if self._store.has('sensor','patch'):
            map = self._store.get('mapper','map')
            cmap = self._store.get('mapper','cmap')
            self._store.lock('mapper')
            map.merge_patch(self._store.get('sensor','patch'))
            cmap.update_critical_map()
            self._store.release('mapper')

    def destroy(self):
        self.__threadw__.stop()

