import cv2
from controller.wrappers import ThreadWrapper
from controller.structures import Map,CriticalMap


class Mapper:

    def __init__(self,store):
        self.__threadw__ = ThreadWrapper(self.__callback__,store.get('general','mapper_node_callback_interval'))
        self.__threadw__.__thread__.setName(store.get('names','namespace')+'/mapper')
        map = Map(store.get('general','sensor_radius'),store.get('general','map_resolution'),store.get('general','map_obstacle_lookup_radius'))
        cmap = CriticalMap(map,store.get('general','critical_radius'),store.get('general','hyper_critical_radius'))
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