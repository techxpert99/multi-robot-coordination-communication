from os import cpu_count
from threading import Thread
from time import sleep, time_ns
from controller.mover import Mover
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import LaserScan

from rclpy.qos import qos_profile_sensor_data

from rclpy import init,shutdown

from controller.stores import SafeParameterStore
from controller.estimator import EstimatorNode
from controller.sensor import SensorNode
from controller.controller import ControllerNode
from controller.mapper import Mapper
from controller.critical_controller import CriticalController
from controller.planner import LocalPlanner
from controller.visualizer import Visualizer


def status_printer(store):
    global gl_status_thread
    from controller.stores import print_status
    def state_print():
        while not store.get('general','executor')._is_shutdown:
            print_status()
            sleep(5)
            
    gl_status_thread = Thread(target=state_print,name="status-printer-thread")
    gl_status_thread.start()

def initialize_exit_point(store):

    global gl_exit_thread

    def exit_fn():
        print('Enter goals as {x,y}. Anything else to exit')
        while True:
            try:
                x,y = (float(_) for _ in input().split(','))
                store.lock('planner')
                store.unsafe_set('planner','plan',[])
                store.unsafe_set('planner','goal',(x,y))
                store.release('planner')
            except:
                break
        print('Acknowledged')
        store.set('general','shutdown',True)

    from threading import Thread
    gl_exit_thread = Thread(target=exit_fn)
    gl_exit_thread.start()


from controller.stores import print_status

def entry_point():
    
    TOPIC_MAP = {'laser.in': ('/demo/laser/out',LaserScan,qos_profile_sensor_data), 'odom.in': ('/demo/odom',Odometry,10), 'vel.out':('/demo/cmd_vel',Twist,10)}

    STORE = SafeParameterStore()

    STORE.register('general')
    STORE.register('critical')
    STORE.register('estimator')
    STORE.register('controller')
    STORE.register('mapper')
    STORE.register('sensor')
    STORE.register('planner')
    STORE.register('visualizer')
    STORE.register('names')

    STORE.set('general','topics',TOPIC_MAP)

    STORE.set('general','critical_radius',0.5)
    STORE.set('general','hyper_critical_radius',1.0)
    STORE.set('general','sensor_radius',9.0)

    STORE.set('general','initial_velocity',(0.0,0.0))
    STORE.set('general','map_resolution',0.3)
    STORE.set('general','map_obstacle_lookup_radius',0.3)
    STORE.set('general','planner_path_analysis_radius',0)

    STORE.set('general','critical_node_callback_interval',0.05)
    STORE.set('general','mapper_node_callback_interval',0.1)
    STORE.set('general','planner_node_callback_interval',0.1)
    STORE.set('general','visualizer_node_callback_interval',0.1)
    STORE.set('general','mover_node_callback_interval',0.1)

    STORE.set('names','namespace','robot1')
    STORE.set('names','estimator','estimator')
    STORE.set('names','sensor','sensor')
    STORE.set('names','controller','controller')

    init()

    EXECUTOR = MultiThreadedExecutor(cpu_count() or 4)
    STORE.set('general','executor',EXECUTOR)
    
    ESTIMATOR_NODE = EstimatorNode(STORE)

    SENSOR_NODE = SensorNode(STORE)

    CONTROLLER_NODE = ControllerNode(STORE)

    CRITICAL_NODE = CriticalController(CONTROLLER_NODE,STORE)

    MAPPER_NODE = Mapper(STORE)

    PLANNER_NODE = LocalPlanner(CONTROLLER_NODE,STORE)

    VISUALIZER_NODE = Visualizer(STORE)

    MOVER_NODE = Mover(STORE,CONTROLLER_NODE)

    initialize_exit_point(STORE)
    #status_printer(STORE)

    while not EXECUTOR._is_shutdown:
        EXECUTOR.spin_once(0.1)

    MOVER_NODE.destroy()

    VISUALIZER_NODE.destroy()

    PLANNER_NODE.destroy()

    MAPPER_NODE.destroy()

    CRITICAL_NODE.destroy()

    CONTROLLER_NODE.destroy()

    SENSOR_NODE.destroy()

    ESTIMATOR_NODE.destroy()

    #EXECUTOR.shutdown()
    #gl_status_thread.join()

    shutdown()


if __name__ == '__main__':
	entry_point()