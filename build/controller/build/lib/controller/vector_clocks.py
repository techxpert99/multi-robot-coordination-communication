from threading import Lock


class VectorClock:
    
    def __init__(self,current_process,initial_process_list=set()):
        self._clock = {process:0 for process in initial_process_list}
        self._clock[current_process] = 0
        self._self = current_process
    
    def increment(self):
        self._clock[self._self] += 1
    
    def expand(self,new_process_list):
        for process in new_process_list:
            if process not in self._clock:
                self._clock[process] = 0
    
    def update(self,new_clock):
        process_vector,time_vector = new_clock
        for i,process in enumerate(process_vector):
            if process in self._clock:
                self._clock[process] = max(self._clock[process],time_vector[i])
            else:
                self._clock[process] = time_vector[i]

    def serialize(self):
        process_vector = []
        time_vector = []
        for process,time in self._clock.items():
            process_vector.append(process)
            time_vector.append(time)
        return (process_vector,time_vector)
    
    def copy(self):
        v = VectorClock(self._self)
        v._clock = self._clock.copy()
        return v
    
    def compare_with(self,clock):
        result = 0
        for id,c1 in self._clock.items():
            if id not in clock:
                c2 = 0
            else:
                c2 = clock[id]
            if result == 0:
                if c1 < c2:
                    result = -1
                elif c1 > c2:
                    result = 1
            elif (result == -1 and c1>c2) or (result == 1 and c1<c2):
                return 0
        return result
    
    def happens_before(self,clock): return self.compare_with(clock) < 0

    def happens_after(self,clock): return self.compare_with(clock) > 0

    def concurrent_to(self,clock): return self.compare_with(clock) == 0

    def strict_compare_with(self,clock):
        result = self.compare(clock)
        if result == 0:
            if self._self < clock._self: return -1
            elif self._self > clock._self: return 1
            return 0
        return result
    
    def strict_happens_before(self,clock): return self.strict_compare_with(clock) < 0

    def strict_happens_after(self,clock): return self.strict_compare_with(clock) > 0

    def equal_to(self,clock): return self.strict_compare_with(clock) == 0


class SafeVectorClock(VectorClock):
    
    def __init__(self,process,initial_process_set=set()):
        VectorClock.__init__(self,process,initial_process_set)
        self._lock = Lock()
    
    def safe_increment(self):
        self._lock.acquire()
        VectorClock.increment(self)
        self._lock.release()
    
    def safe_update(self,new_clock):
        self._lock.acquire()
        VectorClock.update(self,new_clock)
        self._lock.release()
    
    def safe_serialize(self):
        self._lock.acquire()
        data = VectorClock.serialize(self)
        self._lock.release()
        return data
    
    def safe_copy(self):
        self._lock.acquire()
        cp = VectorClock.copy(self)
        self._lock.release()
        return cp
    
    def lock(self):
        self._lock.acquire()
    
    def release(self):
        self._lock.release()