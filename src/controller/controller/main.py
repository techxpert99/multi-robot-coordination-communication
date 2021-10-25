from os import cpu_count
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


def initialize_exit_point(store):

    global gl_exit_thread
    
    def exit_fn():
        print('Enter goals as {x,y}. Anything else to exit')
        while True:
            try:
                x,y = (float(_) for _ in input().split(','))
                store.set('general','goal',(x,y))
            except:
                break
        print('Acknowledged')

    from threading import Thread
    gl_exit_thread = Thread(target=exit_fn)
    gl_exit_thread.start()


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

    STORE.set('general','topics',TOPIC_MAP)

    init()

    EXECUTOR = MultiThreadedExecutor(cpu_count() or 4)
    STORE.set('general','executor',EXECUTOR)
    
    ESTIMATOR_NODE = EstimatorNode(STORE)

    SENSOR_NODE = SensorNode(STORE)

    CONTROLLER_NODE = ControllerNode(STORE)

    CRITICAL_NODE = CriticalController(CONTROLLER_NODE,STORE)

    MAPPER_NODE = Mapper(STORE,critical_radius=1.5,merge_interval=0.1)

    PLANNER_NODE = LocalPlanner(CRITICAL_NODE,STORE,)

    VISUALIZER_NODE = Visualizer(STORE,interval=100)

    MOVER_NODE = Mover(STORE,CONTROLLER_NODE)

    initialize_exit_point(STORE)

    while gl_exit_thread.is_alive():
        EXECUTOR.spin_once(0.1)

    MOVER_NODE.destroy()

    VISUALIZER_NODE.destroy()

    PLANNER_NODE.destroy()

    MAPPER_NODE.destroy()

    CRITICAL_NODE.destroy()

    CONTROLLER_NODE.destroy()

    SENSOR_NODE.destroy()

    ESTIMATOR_NODE.destroy()

    EXECUTOR.shutdown()

    shutdown()


if __name__ == '__main__':
	entry_point()