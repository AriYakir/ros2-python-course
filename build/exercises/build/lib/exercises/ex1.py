# Copyright 2016 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

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
        
        # Poses for the two turtles
        # turtle1 is stationary 
        # turtle2 should catch it (this is your turtle)
        # These two poses should be updated in order to keep track of where the turtles are
        self.turtle1_pose = Pose()
        self.turtle2_pose = Pose()

        # Spawning a second turtle (turtle2)
        # Here we connect to the service and wait for it to become available
        self.cli1 = self.create_client(Kill, 'kill')
        while not self.cli1.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req1 = Kill.Request(name="turtle2")
        self.cli2 = self.create_client(Spawn, 'spawn')
        while not self.cli2.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req2 = Spawn.Request()

        
        ###############################################
        # 1. --> your code here
        # Create here a subscription to /turtle1/pose 
        # Create a callback, turtle1_pose_update 
        # In it, update the pose of the turtle you want to catch (turtle1)
        
        #
        #--> end of your code 
        ################################################

        
        # Here I create a subscription to /turtle2/pose 
        # In it, I update the pose of the target turtle
        self.subscription_2 = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.turtle2_pose_update,
            10)
        self.subscription_2  # prevent unused variable warning
 
        
        
        ###############################################
        # 3. --> your code here
        # Create a publisher to publish to /turtle2/cmd_vel
        # Call it publisher_predator 
        # You will use it later in the game to move your predator turtle
        pass 
        #
        #--> end of your code 
        ################################################


        # This is a function called periodically in which the game logic should be implemented
        game_timer_period = 0.2     # seconds
        self.game_counter = 0               # counter 
        self.game_timer = self.create_timer(game_timer_period, self.game)
        
        # This is a function called periodically to monitor the game progress
        game_monitor_timer_period = 0.2     # seconds
        self.game_monitor_timer = self.create_timer(game_monitor_timer_period, self.game_monitor)

    # Method for spawning a turtle
    def kill_request(self):
        print("killing your turtle!")
        self.req1.name = "turtle2"
        
        self.future = self.cli1.call_async(self.req1)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    # Method for spawning a turtle
    def spawn_request(self):
        print("Spawning your turtle!")
        self.req2.name = "turtle2"
        self.req2.x = random.uniform(0, 10.0)
        self.req2.y = random.uniform(0, 10.0)

        self.future = self.cli2.call_async(self.req2)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()
        
    
    def turtle1_pose_update(self, data):
        ###############################################
        # Here update the location of turtle 1
        # 2. --> your code here
        pass
        # --> end of your code 
        ################################################


    def turtle2_pose_update(self, data):
        self.turtle2_pose = data
            
    
    def distance(self, from_pose, to_pose):
        return self.euclidean_distance(from_pose, to_pose)

    def euclidean_distance(self, from_pose, to_pose):
        """Euclidean distance between from->to pose."""
        return math.sqrt(
                    math.pow((to_pose.x - from_pose.x), 2) +
                    math.pow((to_pose.y - from_pose.y), 2)   )

    def angular(self, from_pose, to_pose):
        return self.steering_angle(from_pose,to_pose) - from_pose.theta

    
    # The (directed) angle from vector1 to vector2
    def steering_angle(self, from_pose, to_pose):
        return math.atan2(to_pose.y - from_pose.y, to_pose.x - from_pose.x)

        

    def degrees2radians(self, angle_in_degrees):
	    return angle_in_degrees * ( math.pi / 180.0 )


    def game_monitor(self):
        if self.euclidean_distance(self.turtle1_pose,self.turtle2_pose) < 0.1:
            print("Kawabanga!!!!!")
            raise SystemExit
        else:
            if self.game_counter < 20:
                print("Catch me if you can...")
            else:
                print("Too slow, try again")
                raise SystemExit
            self.game_counter += 1

    def game(self):
        ###############################################
        # 4. --> your code here
        # You can print the target location
        # self.get_logger().info('target location: x="%s" y="%s"' % (self.turtle1_pose.x,self.turtle1_pose.y))      
        # use here your publisher publisher_predator to move your turtle
        pass
        # --> end of your code 
        ################################################



def main(args=None):
    rclpy.init(args=args)

    moveTurtle = MoveTurtle()

    # Spawn goal turtle
    goal_turtle = moveTurtle.kill_request()

    # Spawn goal turtle
    goal_turtle = moveTurtle.spawn_request()

    rclpy.spin(moveTurtle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    moveTurtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
