from threading import Lock, Thread
#from controller.robot import Robot
#from controller.auxiliary import wait
#from controller.configuration_parser import read_conf
#from controller.global_communicator import GlobalCommunicator
from controller.controller import Controller
import rclpy
import cv2

def cv2_thread_start():
    global vthrd,vlock,vrun,vdata
    def cb():
        prev = None
        while True:
            vlock.acquire()
            if not vrun:
                vlock.release()
                break
            if vdata['window_name'] is not None and vdata['data'] is not None:
                cv2.imshow(vdata['window_name'],vdata['data'])
                cv2.waitKey(100)
                vlock.release()
            else:
                vlock.release()
                wait(0.1)
        cv2.destroyAllWindows()    
    vlock = Lock()
    vrun = True
    vdata = {'window_name':None,'data':None,'lock':vlock}
    vthrd = Thread(target=cb)
    vthrd.start()

def cv2_thread_end():
    global vrun
    vlock.acquire()
    vrun = False
    vlock.release()
    vthrd.join()

def global_communication_start():
    global comnode
    comnode = GlobalCommunicator()

def global_communication_end():
    comnode.destroy()

inth = None
goal = None
target_lock = Lock()
in_lock = Lock()
def InpuThreadStart():
    global inth,flag
    def infunc():
        global goal,flag
        while True:
            in_lock.acquire()
            this_flag = flag
            in_lock.release()
            if not this_flag: break
            this_target = input()
            if this_target == 'q':
                flag = False
            else:
                try:
                    temp = this_target.split(',')
                    target_lock.acquire()
                    goal = (float(temp[0]),float(temp[1]),None)
                    target_lock.release()
                except:
                    flag = False
    flag = True            
    inth = Thread(target=infunc)
    inth.start()

def entry_point():
    global goal
    rclpy.init()
    robot_namespace = '/bot1'
    InpuThreadStart()
    controller = Controller(None,robot_namespace)
    
    while True:
        target_lock.acquire()
        my_goal = goal
        goal = None
        target_lock.release()
        if my_goal is not None:
            controller.goal = my_goal
            controller.plan_controller_state = 1
        controller.RunController()
        in_lock.acquire()
        this_flag = flag
        in_lock.release()
        if not this_flag:
            break
    
    inth.join()
    controller.DestroyController()
    rclpy.shutdown()
    
    return
    cv2_thread_start()

    global_communication_start()

    c = read_conf()
    robots = []

    if type(c['spawn_robots']) == str:
        bot_names = [c['spawn_robots']]
    else:
        bot_names = c['spawn_robots']
    for bot in bot_names:
        robots.append(Robot(bot,vdata))
    for bot in robots:
        bot.start()

    while True:
        try:
            line = input()
            i = int(line.split(',')[0])
            x,y = (float(_) for _ in line.split(',')[1:])
            robots[i].set_goal((x,y))
        except:
            try:
                x,y = (float(_) for _ in line.split(','))
                robots[0].set_goal((x,y))
            except:
                print('Acknowledged')
                break

    for bot in robots:
        bot.destroy()

    global_communication_end()
    cv2_thread_end()

if __name__ == '__main__':
    entry_point()