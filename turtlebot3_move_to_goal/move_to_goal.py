import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import time
import math


# "GoForward" class inherits from the base class "Node"
class GoForward(Node):

    def __init__(self):
        # Initialize the node
        super().__init__('Turtle_Bot_3_Move_To_Goal')
        # Initialize the publisher
        self.cmd_publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.subscription = self.create_subscription(Odometry,'/odom',
            self.odom_callback, 10)
        self.subscription  # prevent unused variable warning
        self.Current_goal_number = 0
        self.goal_list = [{"x":0, "y":0, "theta": 0}]
        self.x =0.0
        self.y =0.0
        self.yaw=0.0

    def main(self):
        print("main")

    def odom_callback(self, msg):
        print("updating pose info")
        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.x
        q0 = msg.pose.pose.orientation.w
        q1 = msg.pose.pose.orientation.x
        q2 = msg.pose.pose.orientation.y
        q3 = msg.pose.pose.orientation.z
        self.yaw = math.degrees(math.atan2(2.0*(q0*q3 + q1*q2),(1.0-2.0*(q2*q2 + q3*q3))))
     
    
        
    def move_turtlebot(self, linear,angular):
        # Create an object of msg type Twist() 
        # and define linear velocity and angular velocity
        move_cmd = Twist()
        move_cmd.linear.x = 1.0*linear
        move_cmd.angular.z = 1.0*angular
        #publish  the velcity command
        self.cmd_publisher_.publish(move_cmd)
        #log details of the current phase of execution
        self.get_logger().info('Publishing cmd_vel')
        
    def stop_turtlebot(self):
        # define what happens when program is interrupted
        # log that turtlebot is being stopped
        self.get_logger().info('stopping turtlebot')
        # publishing an empty Twist() command sets all velocity components to zero
        # Otherwise turtlebot keeps moving even if command is stopped
        self.cmd_publisher_.publish(Twist())

    
        
    
def main(args=None):
    rclpy.init(args=args)
    
    # we are using try-except tools to  catch keyboard interrupt
    try:
        # create an object for GoForward class
        turtlebot = GoForward()
        # continue untill interrupted
        # rclpy.spin(cmd_publisher)
        linear = 0.2
        angular =0.0
        turtlebot.move_turtlebot( linear,angular)
        time.sleep(1)
        turtlebot.stop_turtlebot( )
        time.sleep(1)
        turtlebot.move_turtlebot( linear,angular)
        time.sleep(1)
        turtlebot.stop_turtlebot( )
        time.sleep(1)
        rclpy.spin(turtlebot)
        # while(True):
        #     print(" ")

        

        
    except KeyboardInterrupt:
        # execute shutdown function
        turtlebot.stop_turtlebot()
        # clear the node
        turtlebot.destroy_node()
        rclpy.shutdown()
        
if __name__ == '__main__':
    main()
