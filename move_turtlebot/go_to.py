#!/usr/bin/env python

from argparse import ArgumentParser
from sys import argv

import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt

parser = ArgumentParser()

parser.add_argument('coordinates')
parser.add_argument('-d', '--distance_tolerance', default=0.1)
parser.add_argument('-o', '--orientation')
parser.add_argument('-a', '--angular_tolerance', default=0.1)
parser.add_argument('-i', '--use_i_controller', action='store_true')

arguments = parser.parse_args()

if arguments.coordinates.count(',') != 1:
  raise ValueError("coordinates must be given as x,y")

arguments.coordinates = [float(coordinate)
                         for coordinate
                         in arguments.coordinates.split(',')]

class MoveTo(Node):

    def __init__(self):

        super().__init__(node_name = 'turtlebot_controller')

        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)

        self.pose_subscriber = \
            self.create_subscription(msg_type    = Odometry,
                                     topic       = '/odom',
                                     callback    = self.update_pose,
                                     qos_profile = 10)

        TIMER_PERIOD = 0.1

        self.state = "idle"

        self.goal = Odometry()

        self.goal.pose.pose.position.x = arguments.coordinates[0]
        self.goal.pose.pose.position.y = arguments.coordinates[1]

        if arguments.orientation:
          self.goal.pose.pose.orientation.z = float(arguments.orientation)

        self.distance_tolerance = arguments.distance_tolerance
        self.angular_tolerance  = arguments.angular_tolerance

        self.I_value = 0.0

        self.timer = \
            self.create_timer(timer_period_sec = TIMER_PERIOD,
                              callback         = self.move_to_position)

        self.pose = Odometry()
        
    def update_pose(self, data):        
        """called when new Odometry message Odometry received by subscriber"""

        self.pose = data
        self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
        self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)

    def euclidean_distance(self):
        """returns distance between current pose and goal"""

        return float(sqrt(pow((self.goal.pose.pose.position.x - self.pose.pose.pose.position.x), 2) +
                    pow((self.goal.pose.pose.position.y - self.pose.pose.pose.position.y), 2)))

    def linear_velocity(self, constant_P = 0.2, constant_I = 0.01):
        """proportional controller for linear velocity control"""

        velocity = self.PI_controller(self.euclidean_distance(),
                                      constant_P,
                                      constant_I)

        velocity = min(velocity,  0.4)
        velocity = max(velocity, -0.4)

        return velocity

    def steering_angle(self):
        """returns angle of path from robot location to destination"""

        return atan2((self.goal.pose.pose.position.y
                      - self.pose.pose.pose.position.y),
                     (self.goal.pose.pose.position.x
                      - self.pose.pose.pose.position.x))

    def angular_velocity(self, constant_P=1, constant_I=0.1):
        """proportional controller to orient robot toward destination"""

        omega = \
            self.PI_controller(delta = (self.steering_angle()
                                        - self.pose.pose.pose.orientation.z),
                               gain_P = constant_P,
                               gain_I = constant_I)

        omega = min(omega,  0.5)
        omega = max(omega, -0.5)

        return omega

    def angular_difference(self):

        delta = (self.pose.pose.pose.orientation.z
                 - self.goal.pose.pose.orientation.z)

        return delta

    def P_controller(self, delta, gain):
        P_value = gain * delta
        return P_value

    def PI_controller(self, delta, gain_P, gain_I):

        self.I_value = self.I_value + delta * gain_I

        self.I_value = min(self.I_value,  0.3)
        self.I_value = max(self.I_value, -0.3)

        if arguments.use_i_controller:
            return self.I_value + self.P_controller(delta, gain_P)
        else:
            return self.P_controller(delta, gain_P)

    def move_to_position(self):

        if self.state == "idle":
            self.state = "translation"

        message = Twist()

        if self.state == "translation":

            if (self.euclidean_distance()
                >= self.distance_tolerance):

                message.linear.x = self.linear_velocity()
                message.linear.y = 0.0
                message.linear.z = 0.0

                message.angular.x = 0.0
                message.angular.y = 0.0
                message.angular.z = self.angular_velocity()

                self.velocity_publisher.publish(message)

            else: # stop after destination reached

                message.linear.x = 0.0
                message.angular.z = 0.0
                self.velocity_publisher.publish(message)

                if arguments.orientation:
                    self.state = "rotation"
                else:
                    exit()

        if self.state == "rotation":

            if (   self.angular_difference() >  self.angular_tolerance
                or self.angular_difference() < -self.angular_tolerance):

                message.angular.z = 0.5 * self.angular_difference()

                self.velocity_publisher.publish(message)

            else:
                message.angular.z = 0.0
                self.velocity_publisher.publish(message)
                exit()

def main(args=None):

    rclpy.init(args=args)

    try:
        rclpy.spin(MoveTo())
        
    except KeyboardInterrupt:
        rclpy.shutdown()
    
if __name__ == '__main__':
    main()
