from controller.wrappers import NodeWrapper
from controller.auxiliary import wait,velocity_to_message,normalize_angle,sgn
from math import pi,sqrt,floor,atan2


class ControllerNode:
    
    def __init__(self, store):
        self.__wrapper__ = NodeWrapper('controller',store)
        self.__publisher__ = self.__wrapper__.create_publisher('vel.out')
        store.set('controller','velocity',store.get('general','initial_velocity'))
        self._store = store

    def control_velocity(self,velocity,num_steps=1,step_interval=1,critical=False):
        initial = self._store.get('controller','velocity')
        Dlin = velocity[0]-initial[0]
        Dang = velocity[1]-initial[1]
        dlin,dang = Dlin/num_steps, Dang/num_steps
        for _ in range(num_steps):
            if not critical and self._store.get('critical','interrupt'):
                return True
            initial = (initial[0] + dlin, initial[1] + dang)
            self.__publisher__.publish(velocity_to_message(initial[0], initial[1]))
            self._store.set('controller','velocity',initial)
            wait(step_interval)
    
    def stop(self,num_steps=1,step_interval=1):
        self.control_velocity((0,0),num_steps,step_interval)
    
    def rotate_by(self, angle):
        
        def yaw():
            return self._store.get('estimator','position')[2]
        
        def diff():
            return normalize_angle(t1-yaw())
        
        def initial_accelerate():
            _,vlin = self._store.get('controller','velocity')
            return self.control_velocity((vlin,w*sgn(initial_diff)),n,s)
        
        def midway_monitor():
            interval = 0.01
            while abs(diff()) >= resolution:
                if self._store.get('critical','interrupt'):
                    return True
                wait(interval)
        
        def final_decelerate():
            _,vlin = self._store.get('controller','velocity')
            return self.control_velocity((vlin,0),n,s)

        t1 = normalize_angle(yaw() + angle)
        w,n,s = 0.2,1,0
        resolution = pi/180
        initial_diff = diff()
        if abs(initial_diff) < resolution: return
        if initial_accelerate() or midway_monitor() or final_decelerate() or self._store.get('critical','interrupt'): return True

    def rotate_to(self,yaw):
        _,_,t = self._store.get('estimator','position')
        return self.rotate_by(yaw-t)

    def move_to(self, point):

        def pos():
            return self._store.get('estimator','position')
        
        def diff():
            x0,y0,t0 = pos()
            x1,y1 = point
            dx,dy = x1-x0,y1-y0
            r = normalize_angle(atan2(dy,dx)-t0)
            d = sqrt(dx*dx+dy*dy)
            return d,r
        
        def reorient():
            r,t = diff()
            return self.rotate_by(t)

        def select_speed():
            s = 0.1
            f = 0.5
            c1,c2,c3,c4 = 0.12,1.384,0.05,0.5
            r,t = diff()
            if r == 0: return None
            for i in range(10,-1,-1):
                v = i*c3
                n = floor(i*c4)
                c = c1*v+c2
                dmin = c*n*s*v
                if dmin <= f*r:
                    break
            if dmin == 0: return None
            return v,n,s,dmin
        
        def initial_accelerate():
            return self.control_velocity((v,0),n,s)

        def midway_move():
            interval = 0.1
            while True:
                if self._store.get('critical','interrupt'):
                    return True
                r,t = diff()
                if r <= dmin/2:
                    break
                if abs(t) >= 10*pi/180:
                    if final_decelerate(): return True
                    return self.move_to(point)
                wait(interval)

        def final_decelerate():
            return self.control_velocity((0,0),n,s)
        
        resolution = 0.1
        r0,t0 = diff()
        if r0 < resolution: return
        data = select_speed()
        if data is None: return
        v,n,s,dmin = data

        if reorient() or initial_accelerate() or midway_move() or final_decelerate() or self._store.get('critical','interrupt'): return True

        return False

    def critical_stop(self):
        self.control_velocity((0.0,0.0),4,0.05,True)

    def destroy(self):
        self.__wrapper__.destroy()
    