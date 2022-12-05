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
        
        self.turtle1_pose = Pose()
        self.turtle2_pose = Pose()

        self.cli = self.create_client(Spawn, 'spawn')
        
        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.req = Spawn.Request()

        
        self.subscription_1 = self.create_subscription(
            Pose,
            '/turtle1/pose',
            self.turtle1_pose_update,
            10)
        self.subscription_1  # prevent unused variable warning

        self.subscription_2 = self.create_subscription(
            Pose,
            '/turtle2/pose',
            self.turtle2_pose_update,
            10)
        self.subscription_2  # prevent unused variable warning


        
        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        self.publisher_predator = self.create_publisher(
            Twist, 
            '/turtle2/cmd_vel', 
            10)
        
        game_timer_period = 0.1  # seconds
        self.game_timer = self.create_timer(game_timer_period, self.game)
        
        

    def turtle1_pose_update(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.turtle1_pose = data
        self.turtle1_pose.x = round(self.turtle1_pose.x, 4)
        self.turtle1_pose.y = round(self.turtle1_pose.y, 4)

    def turtle2_pose_update(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.turtle2_pose = data
        self.turtle2_pose.x = round(self.turtle2_pose.x, 4)
        self.turtle2_pose.y = round(self.turtle2_pose.y, 4)
        if self.euclidean_distance(self.turtle1_pose,self.turtle2_pose) < 0.1:
            print("Kawabanga!!!!!")
            raise SystemExit

    
    def linear_vel(self, from_pose, to_pose, constant=1.5):
        return constant * self.euclidean_distance(from_pose, to_pose)

    def steering_angle(self, from_pose, to_pose):
        return math.atan2(to_pose.y - from_pose.y, to_pose.x - from_pose.x)

    def angular_vel(self, from_pose, to_pose, constant=6):
        return constant * (self.steering_angle(from_pose,to_pose) - from_pose.theta)

        
    def euclidean_distance(self, from_pose, to_pose):
        """Euclidean distance between from->to pose."""
        return math.sqrt(
                    math.pow((to_pose.x - from_pose.x), 2) +
                    math.pow((to_pose.y - from_pose.y), 2)   )

    def send_request(self):
        print("Spawning a turtle")
        self.req.name = "turtle2"
        self.req.x = random.uniform(-3.0, 3.0)
        self.req.y = random.uniform(-3.0, 3.0)

        self.future = self.cli.call_async(self.req)
        rclpy.spin_until_future_complete(self, self.future)
        return self.future.result()


    def game(self):
        #self.get_logger().info('location: "%s" "%s"' % (pose.x, pose.y))
        
        #self.publisher_.publish(twist)
              
       
        """Moves the turtle to the goal."""
        if self.euclidean_distance(self.turtle1_pose,self.turtle2_pose) >= 0.001:
            print("following")
            goal_pose = self.turtle1_pose
            distance_tolerance = 0
            # Get the input from the user.
            goal_pose.x = self.turtle1_pose.x
            goal_pose.y = self.turtle1_pose.y
   
            vel_msg = Twist()

            # Porportional controller.
            # https://en.wikipedia.org/wiki/Proportional_control

            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.linear_vel(self.turtle2_pose,goal_pose)
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel(self.turtle2_pose,goal_pose)

            # Publishing our vel_msg
            self.publisher_predator.publish(vel_msg)
        else:

            vel_msg = Twist()
            # Linear velocity in the x-axis.
            vel_msg.linear.x = 0.0
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = 0.0

            # Publishing our vel_msg
            self.publisher_predator.publish(vel_msg)   



def main(args=None):
    rclpy.init(args=args)

    moveTurtle = MoveTurtle()

    # Spawn goal turtle
    goal_turtle = moveTurtle.send_request()

    rclpy.spin(moveTurtle)

    # Destroy the node explicitly
    # (optional - otherwise it will be done automatically
    # when the garbage collector destroys the node object)
    moveTurtle.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
