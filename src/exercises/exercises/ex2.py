import rclpy
import math
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Spawn
from turtlesim.srv import Kill
import random

from std_msgs.msg import String


class MoveTurtle(Node):

    def __init__(self):
        
        super().__init__('moveTurtle')
        
        # Poses for turtle
        self.turtle1_pose = Pose()
        

        
        self.subscription_1 = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle1_pose_update,
            10)
        self.subscription_1  # prevent unused variable warning
 
        # This is a function called periodically in which the game logic should be implemented
        self.game_counter = 0       # counter 
        
        # This is a function called periodically to monitor the game progress
        game_monitor_timer_period = 0.2     # seconds
        self.game_monitor_timer = self.create_timer(game_monitor_timer_period, self.game_monitor)
    
    def turtle1_pose_update(self, data):
        self.turtle1_pose = data
            
    def game_monitor(self):
        if self.turtle1_pose.x == 10 and self.turtle1_pose.y == 10:
            print("Kawabanga!!!!!")
            raise SystemExit
        else:
            if self.game_counter < 20:
                print("Not there...")
            else:
                print("Too slow, try again")
                raise SystemExit
            self.game_counter += 1



def main(args=None):
    rclpy.init(args=args)

    moveTurtle = MoveTurtle()

    rclpy.spin(moveTurtle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    moveTurtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
