from controller.robot import Robot
from controller.auxiliary import wait

def entry_point():
    robot1 = Robot('bot1')
    #robot2 = Robot('robot2')
    #robot2.set_goal((5.0,0.0))
    robot1.start()
    #robot2.start()
    while True:
        try:
            x,y = (float(_) for _ in input().split(','))
            robot1.set_goal((x,y))
        except:
            print('Acknowledged')
            break
    robot1.destroy()
    #robot2.destroy()


if __name__ == '__main__':
    entry_point()