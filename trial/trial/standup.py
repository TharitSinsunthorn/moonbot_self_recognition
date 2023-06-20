import rclpy
# import the ROS2 python libraries
from rclpy.node import Node
# import the Twist interface from the geometry_msgs package
from geometry_msgs.msg import Twist
from dynamixel_custom_interfaces.msg import SetPosition
from dynamixel_custom_interfaces.srv import GetPosition
import time

class SimplePublisher(Node):

    def __init__(self):
        # Here you have the class constructor
        # call super() in the constructor to initialize the Node object
        # the parameter you pass is the node name
        super().__init__('simple_publisher')
        # create the publisher object
        # in this case, the publisher will publish on /cmd_vel Topic with a queue size of 10 messages.
        # use the Twist module for /cmd_vel Topic
        self.publisher_ = self.create_publisher(SetPosition, 'set_position', 10)
        # define the timer period for 0.5 seconds
        # timer_period = 0.5
        # # create a timer sending two parameters:
        # # - the duration between 2 callbacks (0.5 seconds)
        # # - the timer function (timer_callback)
        # self.timer = self.create_timer(timer_period, self.timer_callback)

    def move_dynamixels(self):
        rate = self.create_rate(10)  # Adjust the rate as needed

        while rclpy.ok():
            msg = SetPosition()
            msg.id = 3
            msg.position = 0
            # Publish the message to the Topic
            self.publisher_.publish(msg)
            time.sleep(1)


            msg.id = 3
            msg.position = 500
            self.publisher_.publish(msg)
            time.sleep(1)

        
        

    
            
def main(args=None):
    # initialize the ROS communication
    rclpy.init(args=args)
    # declare the node constructor
    simple_publisher = SimplePublisher()
    
    simple_publisher.move_dynamixels()
    # rclpy.spin_once(simple_publisher)
    # pause the program execution, waits for a request to kill the node (ctrl+c)
    rclpy.spin(simple_publisher)
    # Explicity destroys the node
    simple_publisher.destroy_node()
    # shutdown the ROS communication
    rclpy.shutdown()

if __name__ == '__main__':
    main()