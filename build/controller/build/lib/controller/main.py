from threading import Lock, Thread
from controller.robot import Robot
from controller.auxiliary import wait
from controller.configuration_parser import read_conf
import cv2

def cv2_thread_start():
    global vthrd,vlock,vrun,vdata

    def cb():
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
        vlock.acquire()
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


def entry_point():

    cv2_thread_start()

    c = read_conf()
    robots = []

    for bot in c['spawn_robots']:
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
    
    cv2_thread_end()

if __name__ == '__main__':
    entry_point()