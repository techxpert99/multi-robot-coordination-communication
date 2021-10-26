from controller.robot import Robot
from controller.auxiliary import wait
from rclpy import init
from rclpy.context import Context

def entry_point():
    robot1 = Robot('robot1')    
    robot2 = Robot('robot2')
    robot1.set_goal((0.0,5.0))
    robot2.set_goal((5.0,0.0))
    robot1.start()
    robot2.start()
    wait(5)
    robot1.destroy()
    robot2.destroy()


if __name__ == '__main__':
    entry_point()