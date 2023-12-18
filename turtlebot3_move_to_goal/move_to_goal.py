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
        self.main, 10)
    self.subscription  # prevent unused variable warning
    timer_period = 0.01  # seconds
    # Initialize a timer that excutes call back function every 0.5 seconds
    # self.timer = self.create_timer(timer_period, self.main)

    self.Current_goal_number = 1
    self.goal_list = [{"x":0.0, "y":0.0, "yaw": 0.0}]
    self.__x =0.0
    self.__y =0.0
    self.__yaw=0.0
    self.target_distance = 0
    self.target_angle = 0
    

      

  def main(self, msg):
    while True:
      self.odom_callback(msg)

      if self.Current_goal_number <= len(self.goal_list):
        # to update target dis and angle
        self.cal_tar_dis_angle( msg)
        print(" in goal")
        print(self.goal_list)
        self.Current_goal_number+=1
      else:
        condition = 'y'
        while condition == 'y' or condition =='Y':
          while True:
            x = input("Enter the X of the next Goal: ")
            try:
                x = float(x)  # Try to convert the input to a float
                break  # Break out of the loop if successful
            except ValueError:
                print("Invalid input. Please enter a valid number.")
          while True:
            y = input("Enter the Y of the next Goal: ")
            try:
                y = float(y)  # Try to convert the input to a float
                break  # Break out of the loop if successful
            except ValueError:
                print("Invalid input. Please enter a valid number.")
          while True:
            yaw = input("Enter the Yaw of the next Goal: ")
            try:
              yaw = float(yaw)  # Try to convert the input to a float
              break  # Break out of the loop if successful
            except ValueError:
              print("Invalid input. Please enter a valid number.")
          new_goal = {"x":x, "y": y, "yaw": yaw}
          self.goal_list.append(new_goal)
          print("Do you want to add Next goal if yes then press y/Y or not then any key ")
          condition = input()


  def cal_tar_dis_angle(self, msg):
    if self.Current_goal_number <= len(self.goal_list):
      x_now = msg.pose.pose.position.x
      y_now = msg.pose.pose.position.y
      x_target = self.goal_list[self.Current_goal_number-1]["x"]
      y_target = self.goal_list[self.Current_goal_number-1]["y"]
      x1 = x_now
      y1 = y_now
      x2 = x_target
      y2 = y_target
      dis_angle = self.cal_dis_angle(x1, y1, x2, y2)  
      self.target_distance = dis_angle["distance"]
      self.angle = dis_angle["angle"]
      return dis_angle
    else:
      return {"distance": 0.0 , "angle":0.0 }

  #  calculate distance and angle 
  def cal_dis_angle(self,x1, y1, x2, y2):
    x = x2- x1
    y = y2 - y1 
    y_x = y/x
    angle = math.atan (y_x)
    angle  = 57.2958 * angle
    if x2 < x1 :
            angle = angle+180
    distance  = math.sqrt( (x*x) + (y*y) )
    # self.target_distance = distance
    # self.angle = angle
    return {"distance":distance , "angle":angle }

  def odom_callback(self, msg):
    print("updating pose info")
    self.__x = msg.pose.pose.position.x
    self.__y = msg.pose.pose.position.x
    q0 = msg.pose.pose.orientation.w
    q1 = msg.pose.pose.orientation.x
    q2 = msg.pose.pose.orientation.y
    q3 = msg.pose.pose.orientation.z
    self.__yaw = math.degrees(math.atan2(2.0*(q0*q3 + q1*q2),(1.0-2.0*(q2*q2 + q3*q3))))

  def get_x(self):
    return self.__x
  
  def get_y(self):
    return self.__y
  
  def get_y(self):
    return self.__y
      
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
    # linear = 0.2
    # angular =0.0
    # turtlebot.move_turtlebot( linear,angular)
    # time.sleep(1)
    # turtlebot.stop_turtlebot( )
    # time.sleep(1)
    # turtlebot.move_turtlebot( linear,angular)
    # time.sleep(1)
    # turtlebot.stop_turtlebot( )
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
