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
        # Creates a node with name 'turtlebot_controller' and make sure it is a
        # unique node (using anonymous=True).
        #-rospy.init_node('turtlebot_controller', anonymous=True)
        super().__init__('turtlebot_controller')

        # Publisher which will publish to the topic '/turtle1/cmd_vel'.
        #self.velocity_publisher = rospy.Publisher('/turtle1/cmd_vel', Twist, queue_size=10)
        self.velocity_publisher = self.create_publisher(Twist, '/cmd_vel', 10)
        
        # A subscriber to the topic '/turtle1/pose'. self.update_pose is called
        # when a message of type Pose is received.
        #self.pose_subscriber = rospy.Subscriber('/turtle1/pose',
        #                                        Pose, self.update_pose)
        self.pose_subscriber = self.create_subscription(Odometry, '/odom', self.update_pose, 10)

        timer_period = 0.1  # seconds
        self.state = 0
        self.g_p = Odometry()
        self.g_p.pose.pose.position.x = 0.0
        self.g_p.pose.pose.position.y = 0.0
        self.distance_tolerance = 0.1

        # Initialize a timer that excutes call back function every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.move2goal)

        self.pose = Odometry()
        #self.rate = rospy.Rate(10)

    def update_pose(self, data):
        #self.get_logger().info('Pose received')
        """Callback function which is called when a new message of type Pose is
        received by the subscriber."""
        self.pose = data
        self.pose.pose.pose.position.x = round(self.pose.pose.pose.position.x, 4)
        self.pose.pose.pose.position.y = round(self.pose.pose.pose.position.y, 4)

    def euclidean_distance(self, goal_pose):
        """Euclidean distance between current pose and the goal."""
        return float(sqrt(pow((goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x), 2) +
                    pow((goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y), 2)))

    def linear_vel(self, goal_pose, constant=0.2):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        velocity = constant * self.euclidean_distance(goal_pose)
        if velocity > 0.4:
            velocity = 0.4
        elif velocity < -0.4:
            velocity = -0.4
        return velocity

    def steering_angle(self, goal_pose):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        return atan2(goal_pose.pose.pose.position.y - self.pose.pose.pose.position.y, goal_pose.pose.pose.position.x - self.pose.pose.pose.position.x)

    def angular_vel(self, goal_pose, constant=1):
        """See video: https://www.youtube.com/watch?v=Qh15Nol5htM."""
        omega = constant * (self.steering_angle(goal_pose) - self.pose.pose.pose.orientation.z)
        if omega > 0.5:
            omega = 0.5
        elif omega < -0.5:
            omega = -0.5
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

        # Please, insert a number slightly greater than 0 (e.g. 0.01).
        #distance_tolerance = float(0.1)


        vel_msg = Twist()

        if self.state == 1:
            if self.euclidean_distance(goal_pose) >= self.distance_tolerance:

                # Porportional controller.
                # https://en.wikipedia.org/wiki/Proportional_control

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

        # If we press control + C, the node will stop.
        #rospy.spin()

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
