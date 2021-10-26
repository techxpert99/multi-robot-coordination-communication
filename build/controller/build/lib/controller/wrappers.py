from rclpy.node import Node
from threading import Thread,Lock
from controller.auxiliary import wait

class NodeWrapper:

    def __init__(self, name, store):
        self.__node__ = Node(name,namespace=store.get('names','namespace'),context=store.get('general','context'))
        store.get('general','executor').add_node(self.__node__)
        self._store = store

    def create_subscription(self,topic,callback):
        topic_name,topic_type,topic_profile = self._store.get('general','topics')[topic]
        return self.__node__.create_subscription(topic_type,topic_name,callback,topic_profile)
    
    def create_publisher(self,topic):
        topic_name,topic_type,topic_profile = self._store.get('general','topics')[topic]
        return self.__node__.create_publisher(topic_type,topic_name,topic_profile)
    
    def destroy(self):
        self.__node__.destroy_node()


class ThreadWrapper:

    def __init__(self,callback,interval):
        self.__lock__ = Lock()
        self.__run__ = True
        self.__interval__ = interval
        self.__callback__ = callback
        self.__thread__ = Thread(target=self.__target__)
    
    def __target__(self):
        while self.__runnable__():
            self.__callback__()
            wait(self.__interval__)
    
    def __runnable__(self):
        self.__lock__.acquire()
        runnable = self.__run__
        self.__lock__.release()
        return runnable
    
    def start(self):
        self.__thread__.start()

    def stop(self):
        self.__lock__.acquire()
        self.__run__ = False
        self.__lock__.release()
        self.__thread__.join()

