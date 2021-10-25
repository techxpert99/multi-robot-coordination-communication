from threading import Lock


class SafeParameterStore:
    
    def __init__(self):
        self.__store__ = dict()
        self.__lock__ = Lock()
    
    def register(self,name):
        self.__lock__.acquire()
        self.__store__[name] = dict(),Lock()
        self.__lock__.release()
        
    def set(self,store,key,value):
        self.__store__[store][1].acquire()
        self.__store__[store][0][key] = value
        self.__store__[store][1].release()
    
    def get(self,store,key):
        self.__store__[store][1].acquire()
        value = self.__store__[store][0][key]
        self.__store__[store][1].release()
        return value
    
    def has(self,store,key):
        self.__store__[store][1].acquire()
        out = key in self.__store__[store][0]
        self.__store__[store][1].release()
        return out
    
    def lock(self,store):
        self.__store__[store][1].acquire()
    
    def release(self,store):
        self.__store__[store][1].release()

    def unsafe_get(self,store,key):
        return self.__store__[store][0][key]
    
    def unsafe_set(self,store,key,value):
        self.__store__[store][0][key] = value
    
    def is_registered(self,store):
        return store in self.__store__