from threading import Lock

from random import randint
from threading import current_thread

lock_status = dict()
count_erroneous_releases = 0
stat_lock = Lock()

def print_status():
    stat_lock.acquire()
    print('### Lock Status ###')
    print()
    for id in lock_status:
        print('Lock ID:',id)
        print('Lock Requested:',lock_status[id]['requested'])
        print('Lock Acquired:',lock_status[id]['acquired'])
        print('Release Requested:',lock_status[id]['release_requested'])
        print()
    print('##################')
    print()
    stat_lock.release()

def stat_req(id):
    stat_lock.acquire()
    lock_status[id]['requested'].add(current_thread().getName())
    stat_lock.release()

def stat_acq(id):
    stat_lock.acquire()
    lock_status[id]['requested'].remove(current_thread().getName())
    lock_status[id]['acquired'].add(current_thread().getName())
    stat_lock.release()

def stat_rel(id):
    stat_lock.acquire()
    lock_status[id]['acquired'].remove(current_thread().getName())
    lock_status[id]['release_requested'].remove(current_thread().getName())
    stat_lock.release()

def stat_rel_req(id):
    stat_lock.acquire()
    lock_status[id]['release_requested'].add(current_thread().getName())
    stat_lock.release()

class LockWrapper:
    def __init__(self):
        self._l = Lock()
        self._id = randint(0,1000)
        lock_status[self._id] = {'acquired':set(),'requested':set(),'release_requested':set()}
    
    def acquire(self):
        stat_req(self._id)
        self._l.acquire()
        stat_acq(self._id)

    def release(self):
        stat_rel_req(self._id)        
        self._l.release()
        stat_rel(self._id)


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