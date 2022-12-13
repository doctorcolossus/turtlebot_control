#!/usr/bin/env python
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from math import pow, sqrt


class MoveToPoz(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')

        # Publisher which will publish to the topic '/cmd_vel'.        
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Odometry is received.        
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.update_pose, 10)

        timer_period = 0.1  # seconds
        self.state = 0
        self.r = 1.6 # defines radiuss of the circular path
        self.x = 0.0
        self.y = 1.6 # positions the path so that robot initial position is on the path
        self.v = 0.1
        self.p = 1.0 # proportional controler (P controler) gain

        # Initialize a timer that excutes call back function every x seconds
        self.timer = self.create_timer(timer_period, self.move2goal)

        self.pose = Odometry()
        
    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
        self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)
    
    def angular_vel(self):
        distance = sqrt(pow(self.pose.pose.pose.position.x - self.x,2)+pow(self.pose.pose.pose.position.y - self.y,2))-self.r      
        self.get_logger().info('Distance: "%s"' % distance)        
        omega = self.p * distance
        if omega > 0.3:
            omega = 0.3
        elif omega < -0.3:
            omega = -0.3
        return omega

    def move2goal(self):

        if self.state == 0:
            # Get the input from the user.
            self.r = float(input("Set circle radiuss: "))
            self.y = self.r
            #self.x = float(input("Set circle x position: ")) #posibility to change path center point
            #self.y = float(input("Set circle y position: "))
            self.v = float(input("Set robot linear velocity: "))
            self.p = float(input("Set P controler gain: "))            
            self.state = 1   

        vel_msg = Twist()

        if self.state == 1:
            # Linear velocity in the x-axis.
            vel_msg.linear.x = self.v
            vel_msg.linear.y = 0.0
            vel_msg.linear.z = 0.0

            # Angular velocity in the z-axis.
            vel_msg.angular.x = 0.0
            vel_msg.angular.y = 0.0
            vel_msg.angular.z = self.angular_vel()

            # Publishing our vel_msg
            self.velocity_publisher.publish(vel_msg)                           

def main(args=None):
    rclpy.init(args=args)
    try:
        x = MoveToPoz()
        rclpy.spin(x)        
    except KeyboardInterrupt:    	
    	rclpy.shutdown()    
     	
if __name__ == '__main__':
    main()