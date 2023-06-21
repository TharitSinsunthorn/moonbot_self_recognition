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
        
        super().__init__('simple_publisher')
        
        self.publisher_ = self.create_publisher(SetPosition, 'set_position', 10)
        # define the timer period for 0.5 seconds
        timer_period = 0.5
        self.timer = self.create_timer(timer_period, self.move_dynamixels)
        self.t = 50

    def move_dynamixels(self):
        rate = self.create_rate(10)  # Adjust the rate as needed

        # while rclpy.ok():
        #     msg = SetPosition()
        #     msg.id = 3
        #     msg.position = 0
        #     # Publish the message to the Topic
        #     self.publisher_.publish(msg)
        #     time.sleep(1)


        #     msg.id = 3
        #     msg.position = 500
        #     self.publisher_.publish(msg)
        #     time.sleep(1)
        if self.t >= 4096:
            self.t -= 4096
        elif self.t < 0:
            self.t += 4097


        # Publish the desired positions to the /dynamixel/command topic
        msg = SetPosition()
        msg.id = 3
        msg.position = self.t
            
        self.publisher_.publish(msg)
            
        self.t -= 50  # Increment the time variable

    
            
        

    
            
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