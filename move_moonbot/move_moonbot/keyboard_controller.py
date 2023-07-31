import keyboard
import rclpy
from rclpy.node import Node
from std_msgs.msg import String

def main():
    rclpy.init()
    node = rclpy.create_node('keyboard_controller')
    node.get_logger().info("Initialised Keyboard Node")
    pub = node.create_publisher(String, 'cmd_bot', 1)

    rate = node.create_rate(10)

    while rclpy.ok():
        if keyboard.is_pressed('w') or keyboard.is_pressed('W'):
            msg = String()
            msg.data = 'up'
            pub.publish(msg)
            node.get_logger().info("Moving Forward")
        elif keyboard.is_pressed('s') or keyboard.is_pressed('S'):
            msg = String()
            msg.data = 'down'
            pub.publish(msg)
            node.get_logger().info("Moving Backward")
        elif keyboard.is_pressed('q') or keyboard.is_pressed('Q'):
            msg = String()
            msg.data = 'right'
            pub.publish(msg)
            node.get_logger().info("Moving Right")
        elif keyboard.is_pressed('a') or keyboard.is_pressed('A'):
            msg = String()
            msg.data = 'left'
            pub.publish(msg)
            node.get_logger().info("Moving Left")
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()