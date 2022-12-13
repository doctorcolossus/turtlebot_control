#!/usr/bin/env python
#!/usr/bin/env python
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
#from turtlesim.msg import Pose
from math import pow, atan2, sqrt


class MoveToPoz(Node):

    def __init__(self):
        super().__init__('turtlebot_controller')

        # Publisher which will publish to the topic '/cmd_vel'.        
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.        
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.update_pose, 10)

        timer_period = 0.1  # seconds
        self.state = 0
        self.g_p = Odometry()
        self.g_p.pose.pose.position.x = 0.0
        self.g_p.pose.pose.position.y = 0.0
        self.distance_tolerance = 0.1

        # Initialize a timer that excutes call back function every 0.1 seconds
        self.timer = self.create_timer(timer_period, self.move2goal)

        self.pose = Odometry()
        #self.rate = rospy.Rate(10)

    def update_pose(self, data):
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
        self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return float(sqrt(pow((goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x), 2) +
                    pow((goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y), 2)))

    def linear_vel(self, goal_pose, constant=0.2, radiuss=3, h=1.6, k=0):
        #velocity = constant * self.euclidean_distance(goal_pose)
        #velocity = sqrt(pow(self.pose.pose.pose.position.x - h,2)+pow(self.pose.pose.pose.position.y - k,2))-radiuss
        velocity = 0.1
        if velocity > 0.2:
            velocity = 0.2
        elif velocity < -0.2:
            velocity = 0.2
        return velocity

    def steering_angle(self, goal_pose, radiuss=1.6, h=0, k=1.6):
        distance = sqrt(pow(self.pose.pose.pose.position.x - h,2)+pow(self.pose.pose.pose.position.y - k,2))-radiuss       
        self.get_logger().info('Distance: "%s"' % distance)
        return distance

    def angular_vel(self, goal_pose, constant=0.5):        
        omega = constant * (self.steering_angle(goal_pose))
        if omega > 0.3:
            omega = 0.3
        elif omega < -0.3:
            omega = -0.3
        return omega

    def move2goal(self):
        """Moves the turtle to the goal."""
        goal_pose = Odometry()

        if self.state == 0:
            # Get the input from the user.
            self.g_p.pose.pose.position.x = float(input("Set your x goal: "))
            self.g_p.pose.pose.position.y = float(input("Set your y goal: "))

            # Please, insert a number slightly greater than 0 (e.g. 0.01).
            self.distance_tolerance = float(input("Set your tolerance: "))
            self.state = 1

                # Get the input from the user.
        goal_pose.pose.pose.position.x = self.g_p.pose.pose.position.x
        goal_pose.pose.pose.position.y = self.g_p.pose.pose.position.y        

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
                # Stopping our robot after the movement is over.
                vel_msg.linear.x = 0.0
                vel_msg.angular.z = 0.0
                self.velocity_publisher.publish(vel_msg)
                self.state = 0        

def main(args=None):
    rclpy.init(args=args)

    try:
        x = MoveToPoz()
        rclpy.spin(x)
        
    except KeyboardInterrupt:    	
    	rclpy.shutdown()
    
     	
if __name__ == '__main__':
    main()