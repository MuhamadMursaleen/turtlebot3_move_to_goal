import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist

# "GoForward" class inherits from the base class "Node"
class GoForward(Node):

    def __init__(self):
    	# Initialize the node
        super().__init__('GoforwardCmd_publisher')
        # Initialize the publisher
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        timer_period = 0.5  # seconds
        # Initialize a timer that excutes call back function every 0.5 seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
     
    
        
    def timer_callback(self):
        # Create an object of msg type Twist() 
        # and define linear velocity and angular velocity
        move_cmd = Twist()
        move_cmd.linear.x = 0.2
        move_cmd.angular.z = 0.0
        #publish  the velcity command
        self.publisher_.publish(move_cmd)
        #log details of the current phase of execution
        self.get_logger().info('Publishing cmd_vel')
        
    def stop_turtlebot(self):
    	# define what happens when program is interrupted
    	# log that turtlebot is being stopped
    	self.get_logger().info('stopping turtlebot')
    	# publishing an empty Twist() command sets all velocity components to zero
    	# Otherwise turtlebot keeps moving even if command is stopped
    	self.publisher_.publish(Twist())
    	
    
def main(args=None):
    rclpy.init(args=args)
    
    # we are using try-except tools to  catch keyboard interrupt
    try:
    	# create an object for GoForward class
    	cmd_publisher = GoForward()
    	# continue untill interrupted
    	rclpy.spin(cmd_publisher)
    	
    except KeyboardInterrupt:
    	# execute shutdown function
    	cmd_publisher.stop_turtlebot()
    	# clear the node
    	cmd_publisher.destroy_node()
    	rclpy.shutdown()
    	
if __name__ == '__main__':
    main()
