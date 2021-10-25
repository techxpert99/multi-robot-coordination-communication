from controller.wrappers import NodeWrapper
from nav_msgs.msg import Odometry
from controller.auxiliary import euler_from_quaternion


class EstimatorNode:

    def __init__(self,store):
        self.__wrapper__ = NodeWrapper('estimator',store)
        self.__wrapper__.create_subscription('odom.in',self.__odometry_callback__)
        self._store = store
    
    def __odometry_callback__(self, msg: Odometry):
        roll, pitch, yaw = euler_from_quaternion(
        msg.pose.pose.orientation.x,
        msg.pose.pose.orientation.y,
        msg.pose.pose.orientation.z,
        msg.pose.pose.orientation.w)
        self._store.set('estimator','position',(msg.pose.pose.position.x,msg.pose.pose.position.y,yaw))

    def destroy(self):
        self.__wrapper__.destroy()
