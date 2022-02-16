from controller.wrappers import ThreadWrapper


class CriticalController:
    def __init__(self,controller,store):
        self._store = store
        self._store.set('critical','interrupt',False)
        self.__threadw__ = ThreadWrapper(self.__critical_callback__,store.get('general','critical_node_callback_interval'))
        self.__threadw__.__thread__.setName(store.get('names','namespace')+'/critical-controller')
        self.__controller__ = controller
        self.__threadw__.start()
        self.__critical_radius__ = store.get('general','critical_radius')
    
    def __critical_callback__(self):
        if not self._store.has('sensor','patch'): return
        patch = self._store.get('sensor','patch')
        for dist in patch.__obstacles__:
            if dist <= self.__critical_radius__:
                self.__critical_action__()
                break

    def __critical_action__(self):
        print('critical_interrupt')
        self._store.set('critical','interrupt',True)
        self.__controller__.critical_stop()
        self._store.set('critical','interrupt',False) 

    def destroy(self):
        self.__threadw__.stop()
