from rclpy.node import Node,Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.publisher import Publisher
from rclpy.qos import qos_profile_system_default,qos_profile_sensor_data

from rclpy import init,shutdown
from controller.wrappers import ThreadWrapper
from robot_communication_interfaces.msg import CommunicationMessage

class GlobalCommunicator:
    
    def __init__(self):
        self._communication_radius = 20.0
        self._context = Context()
        init(context=self._context)
        self._tw = ThreadWrapper(self._spin_function,0)
        self._node = Node('communicator_node',namespace='global_robot_communicator',context=self._context)
        self._node.create_subscription(CommunicationMessage,'/global_robot_communicator/broadcasts',self._recv_cb,qos_profile_sensor_data)
        self._executor = SingleThreadedExecutor(context=self._context)
        self._executor.add_node(self._node)
        self._tw.start()
        self._robots = dict()
    
    def _spin_function(self):
        self._executor.spin_once(timeout_sec=0.2)
    
    def _are_near(self,pos1,pos2):
        return ((pos1[0]-pos2[0])**2+(pos1[1]-pos2[1])**2)**0.5 <= self._communication_radius

    def _recv_cb(self,msg: CommunicationMessage):
        
        if msg.robot_id not in self._robots:
            self._robots[msg.robot_id] = [self._node.create_publisher(CommunicationMessage,f'/{msg.robot_id}/communication_receiver',qos_profile_system_default),msg.position]
        else:
            self._robots[msg.robot_id][1] = msg.position
        
        if msg.message_type == 'discovery':
            for robot in self._robots:
                if robot != msg.robot_id and self._are_near(msg.position,self._robots[robot][1]):
                    self._robots[robot][0].publish(msg)
        else:
            if msg.receiver_robot_id in self._robots and self._are_near(msg.position,self._robots[msg.receiver_robot_id][1]):
                self._robots[msg.receiver_robot_id][0].publish(msg)
    
    def destroy(self):
        self._node.destroy_node()
        self._tw.stop()
        self._executor.shutdown()
        shutdown(context=self._context)