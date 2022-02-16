from time import time
from controller.wrappers import ThreadWrapper
from controller.comapper import World
#from controller.structures import Map,CriticalMap
#from controller.cmap_generation import OptimizedCriticalMap

class Mapper:

    def __init__(self,store):
        self.__threadw__ = ThreadWrapper(self.__callback__,store.get('general','mapper_node_callback_interval'))
        self.__threadw__.__thread__.setName(store.get('names','namespace')+'/mapper')
        #map = Map(store.get('general','sensor_radius'),store.get('general','map_resolution'),store.get('general','map_obstacle_lookup_radius'))
        #cmap = CriticalMap(map,store.get('general','critical_radius'),store.get('general','hyper_critical_radius'))
        #cmap2 = OptimizedCriticalMap(store.get('general','sensor_radius'),store.get('general','map_resolution'),store.get('general','map_padding'),store.get('general','map_obstacle_lookup_radius'),store.get('general','critical_radius'),store.get('general','hyper_critical_radius'))
        world = World(store)
        self._store = store
        #self._store.set('mapper','map',map)
        #self._store.set('mapper','cmap',cmap)
        self._store.set('mapper','world',world)
        self.__threadw__.start()
    
    def __callback__(self):
        if self._store.has('sensor','patch'):
            # map = self._store.get('mapper','map')
            # cmap = self._store.get('mapper','cmap')
            self._store.lock('sensor')
            patch = self._store.unsafe_get('sensor','patch').copy()
            self._store.release('sensor')
            # from time import time_ns
            self._store.lock('mapper')
            world = self._store.unsafe_get('mapper','world')
            # s = time_ns()
            # map.merge_patch(patch)
            # cmap.update_critical_map()
            # e = time_ns()
            # f1 = e-s
            # s = time_ns()
            #cmap_tuple, patch, sensor_radius, resolution, padding, obstacle_radius,critical_radius,hypercritical_radius
            
            world.patch(patch)

            # e = time_ns()
            # f2 = e-s
            # print('overhead:',f2/f1)

            self._store.release('mapper')



    def destroy(self):
        self.__threadw__.stop()
        #self._store.lock('mapper')
        #cmap = self._store.unsafe_get('mapper','cmap')
        #from controller.auxiliary import save_critical_map
        #from random import randint
        #save_critical_map(f'/home/ghost/Desktop/dev_ws/src/controller/controller/saved_cmaps/cmap_{randint(0,1000000)}',cmap.get_critical_map())
        #self._store.release('mapper')