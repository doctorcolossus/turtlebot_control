#!/usr/bin/env python
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, atan2, sqrt


class MoveToPoz(Node):

    def __init__(self):
        # Creates a node with name 'turtlebot_controller'        
        super().__init__('turtlebot_controller')

        # Publisher which will publish to the topic '/cmd_vel'.
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # A subscriber to the topic '/pose' with a calback to 'update_pose'
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.update_pose, 10)

        timer_period = 0.1  # seconds
        self.state = 0 # if movement in progress, eaquals to 1
        self.init_theta = "y"
        self.g_p = Odometry()
        self.g_p.pose.pose.position.x = 0.0
        self.g_p.pose.pose.position.y = 0.0
        self.g_p.pose.pose.orientation.z = 0.0
        self.distance_tolerance = 0.1
        self.angular_tolerance = 0.1
        self.I_value = 0.0

        # Initialize a timer that excutes call back function in desired frequency
        self.timer = self.create_timer(timer_period, self.move2goal)

        self.pose = Odometry()
        
    def update_pose(self, data):        
        #Callback function which is called when a new message of type Odometry is
        #received by the subscriber.
        self.pose = data
        self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
        self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)

    def euclidean_distance(self, goal_pose):
        #Distance between current pose and the goal.
        return float(sqrt(pow((goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x), 2) +
                    pow((goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y), 2)))

    def linear_vel(self, goal_pose, constant_P=0.2, constant_I = 0.01):
        #Proportional controler for lin. velocity controll
        #velocity = constant * self.euclidean_distance(goal_pose)
        velocity = self.PI_controller(self.euclidean_distance(goal_pose), constant_P, constant_I)
        if velocity > 0.4:
            velocity = 0.4
        elif velocity < -0.4:
            velocity = -0.4
        return velocity

    def steering_angle(self, goal_pose):
        #Angle of the path from robot location to destination.
        return atan2(goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y, goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x)

    def angular_vel(self, goal_pose, constant_P=1, constant_I=0.1):
        #Proportional controler to align robot to the direction of the destination.
        #omega = constant * (self.steering_angle(goal_pose) - self.pose.pose.pose.orientation.z)
        omega = self.PI_controller((self.steering_angle(goal_pose) - self.pose.pose.pose.orientation.z), constant_P, constant_I)
        if omega > 0.5:
            omega = 0.5
        elif omega < -0.5:
            omega = -0.5
        return omega

    def angular_difference(self, goal_pose):
        delta = self.pose.pose.pose.orientation.z - goal_pose.pose.pose.orientation.z
        return delta

    ############contoler_stuff#################

    def P_controller(self, delta, gain):
        P_value = gain * delta
        return P_value

    def PI_controller(self, Delta, gain_P, gain_I):
        self.I_value = self.I_value + Delta * gain_I
        if self.I_value > 0.3:
            self.I_value = 0.3
        elif self.I_value < -0.3:
            self.I_value = -0.3

        use_I_controler = True #if set to true PI controler wil be used insted of just P

        if use_I_controler:
            return self.I_value + self.P_controller(Delta, gain_P)
        else:
            return self.P_controller(Delta, gain_P)

    ##########################################

    def move2goal(self):
        #Moves the turtle to the goal        
        goal_pose = Odometry()

        if self.state == 0:
            # Get the input from the user.
            self.g_p.pose.pose.position.x = float(input("Set your x goal: "))
            self.g_p.pose.pose.position.y = float(input("Set your y goal: "))
            self.init_theta = input("Is theta a requirement y/n?: ")
            if self.init_theta == "y":
                self.g_p.pose.pose.orientation.z = float(input("Set your theta goal: "))
                self.angular_tolerance = float(input("Set your angular tolerance: "))

            # Insert a number slightly greater than 0 (e.g. 0.1).
            self.distance_tolerance = float(input("Set your tolerance: "))
            self.state = 1

        goal_pose.pose.pose.position.x = self.g_p.pose.pose.position.x
        goal_pose.pose.pose.position.y = self.g_p.pose.pose.position.y
        goal_pose.pose.pose.orientation.z = self.g_p.pose.pose.orientation.z

        vel_msg = Twist()

        if self.state == 1:
            if self.euclidean_distance(goal_pose) >= self.distance_tolerance:               
                # Linear velocity in the x-axis.
                vel_msg.linear.x = self.linear_vel(goal_pose)
                vel_msg.linear.y = 0.0
                vel_msg.linear.z = 0.0

                # Angular velocity in the z-axis.
                vel_msg.angular.x = 0.0
                vel_msg.angular.y = 0.0
                vel_msg.angular.z = self.angular_vel(goal_pose)

                # Publishing our vel_msg
                self.velocity_publisher.publish(vel_msg)

                # Publish at the desired rate.
                #self.rate.sleep()
            else:
                # Stopping our robot after destination is reached.
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)
                if self.init_theta == "y":
                    self.state = 2
                else:
                    self.state = 0
        if self.state == 2:
            if self.angular_difference(goal_pose) > self.angular_tolerance or self.angular_difference(goal_pose) < ((-1)*self.angular_tolerance):
               vel_msg.angular.z = (self.angular_difference(goal_pose) * 0.5)
               self.velocity_publisher.publish(vel_msg)
            else:
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)
                self.state = 0

def main(args=None):
    rclpy.init(args=args)

    try:
        x = MoveToPoz()
        rclpy.spin(x)
        
    except KeyboardInterrupt:
    	# execute shutdown function
    	#move_bobot.stop_turtlebot()
    	# clear the node
    	#move_bobot.destroy_node()
    	rclpy.shutdown()
    
     	
if __name__ == '__main__':
    main()
