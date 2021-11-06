from os import cpu_count
from threading import Lock, Thread
from controller.coco import CoCoNode
from controller.communication import CommunicationNode
from controller.coordination import CoordinationNode
from controller.mover import Mover
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.context import Context
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from sensor_msgs.msg import LaserScan

from rclpy.qos import qos_profile_sensor_data
from rclpy.qos import qos_profile_system_default

from rclpy import init,shutdown

from controller.stores import SafeParameterStore
from controller.estimator import EstimatorNode
from controller.sensor import SensorNode
from controller.controller import ControllerNode
from controller.mapper import Mapper
from controller.critical_controller import CriticalController
from controller.planner import LocalPlanner
from controller.visualizer import Visualizer
from controller.configuration_parser import read_conf
from robot_communication_interfaces.msg import CommunicationMessage


class Robot:

    def __init__(self, name, viz_data_dict):
        
        cfg = read_conf()
        laser_in = '/'+name+'/'+cfg['robot_laser_out']+'/out'
        odom_in = '/'+name+'/'+cfg['robot_odometery']
        vel_out = '/'+name+'/'+cfg['robot_velocity']
        com_out = '/global_robot_communicator/broadcasts'
        com_in = '/'+name+'/communication_receiver'

        TOPIC_MAP = {'laser.in': (laser_in,LaserScan,qos_profile_sensor_data), 'odom.in': (odom_in,Odometry,qos_profile_system_default), 'vel.out':(vel_out,Twist,qos_profile_system_default),'broadcasts':(com_out,CommunicationMessage,qos_profile_sensor_data),'com.in':(com_in,CommunicationMessage,qos_profile_system_default)}

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
        STORE.register('coordinator')
        STORE.register('coco')

        STORE.set('general','topics',TOPIC_MAP)

        STORE.set('general','critical_radius',0.5)
        STORE.set('general','hyper_critical_radius',1.0)
        STORE.set('general','sensor_radius',9.0)

        STORE.set('general','initial_velocity',(0.0,0.0))
        STORE.set('general','map_resolution',0.5)
        STORE.set('general','map_obstacle_lookup_radius',0.5)
        STORE.set('general','planner_path_analysis_radius',0)

        STORE.set('general','critical_node_callback_interval',0.1)
        STORE.set('general','mapper_node_callback_interval',0.5)
        STORE.set('general','planner_node_callback_interval',0.5)
        STORE.set('general','visualizer_node_callback_interval',0.5)
        STORE.set('general','mover_node_callback_interval',0.5)
        STORE.set('general','communication_broadcast_callback_interval',0.5)
        STORE.set('general','coordination_interrupt_handler_callback_interval',0.5)
        STORE.set('general','coordination_exchange_timeout',10.0)
        STORE.set('general','coordination_exchange_spin_interval',0.5)
        STORE.set('general','coordination_exchange_interval',60)
        STORE.set('general','coordination_encounter_handler_callback_interval',1.0)

        STORE.set('general','executor_callback_interval',0.1)

        STORE.set('names','namespace',name)
        STORE.set('names','estimator','estimator')
        STORE.set('names','sensor','sensor')
        STORE.set('names','controller','controller')
        STORE.set('names','communication','communicator')
        STORE.set('names','coco','coco')

        STORE.set('visualizer','window_name',f'Map: {name}')
        STORE.set('visualizer','viz_data_dict',viz_data_dict)
        
        self._MAIN_THREAD = Thread(name=STORE.get('names','namespace')+'/main',target=self.__execution_function__)
        self._LOCK = Lock()
        self._RUNNABLE = True
        self._TOPIC_MAP = TOPIC_MAP
        self._STORE = STORE
       

    def isRunnable(self):
        self._LOCK.acquire()
        runnable = self._RUNNABLE
        self._LOCK.release()
        return runnable

    def start(self):
        self._MAIN_THREAD.start()

    def destroy(self):
        self._LOCK.acquire()
        self._RUNNABLE = False
        self._LOCK.release()
        self._MAIN_THREAD.join()

    def __execution_function__(self):
        
        context = Context()
        init(context=context)

        STORE = self._STORE

        #EXECUTOR = MultiThreadedExecutor(4,context=context)
        EXECUTOR = SingleThreadedExecutor(context=context)
        STORE.set('general','executor',EXECUTOR)

        STORE.set('general','context',context)

        ESTIMATOR_NODE = EstimatorNode(STORE)

        SENSOR_NODE = SensorNode(STORE)

        CONTROLLER_NODE = ControllerNode(STORE)

        CRITICAL_NODE = CriticalController(CONTROLLER_NODE,STORE)

        MAPPER_NODE = Mapper(STORE)

        PLANNER_NODE = LocalPlanner(CONTROLLER_NODE,STORE)

        VISUALIZER_NODE = Visualizer(STORE)

        MOVER_NODE = Mover(STORE,CONTROLLER_NODE)

        #COMMUNICATION_NODE = CommunicationNode(STORE)

        #COORDINATION_NODE = CoordinationNode(COMMUNICATION_NODE,CONTROLLER_NODE,STORE)

        COCO_NODE = CoCoNode(STORE)

        interval = self._STORE.get('general','executor_callback_interval')
        while self.isRunnable():
            EXECUTOR.spin_once(interval)

        executor_shutdown = False
        lock = Lock()

        def final_execution():
            while True:
                lock.acquire()
                if executor_shutdown:
                    lock.release()
                    break
                lock.release()
                EXECUTOR.spin_once(interval)
        
        final_executor = Thread(target=final_execution)
        final_executor.start()

        COCO_NODE.destroy()
        #COORDINATION_NODE.destroy()
        #COMMUNICATION_NODE.destroy()
        MOVER_NODE.destroy()
        VISUALIZER_NODE.destroy()
        PLANNER_NODE.destroy()
        MAPPER_NODE.destroy()
        CRITICAL_NODE.destroy()
        CONTROLLER_NODE.destroy()
        SENSOR_NODE.destroy()
        ESTIMATOR_NODE.destroy()

        lock.acquire()
        executor_shutdown = True
        lock.release()
        final_executor.join()

        EXECUTOR.shutdown()

        shutdown(context=context)
    
    def set_goal(self,goal):
        self._STORE.set('planner','goal',goal)
        self._STORE.set('planner','plan',[])