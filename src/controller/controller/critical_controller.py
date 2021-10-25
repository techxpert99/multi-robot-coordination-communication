from controller.wrappers import ThreadWrapper


class CriticalController:
    def __init__(self,controller,store,critical_interval=0.05,critical_radius=1.0):
        self._store = store
        self._store.set('critical','interrupt',False)
        self.__threadw__ = ThreadWrapper(self.__critical_callback__,critical_interval)
        self.__controller__ = controller
        self.__threadw__.start()
        self.__critical_radius__ = critical_radius
    
    def __critical_callback__(self):
        if not self._store.has('sensor','patch'): return
        patch = self._store.get('sensor','patch')
        for dist in patch.__obstacles__:
            if dist <= self.__critical_radius__:
                self.__critical_action__()
                break

    def __critical_action__(self):
        print('interrupt')
        self._store.set('critical','interrupt',True)
        self.__controller__.control_velocity((0.0,0.0),4,0.05,True)
        self._store.set('critical','interrupt',False) 

    def destroy(self):
        self.__threadw__.stop()
