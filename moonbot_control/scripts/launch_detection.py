#!/usr/bin/env python3

import time
import rclpy
from rclpy.timer import Rate
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor, SingleThreadedExecutor
from rclpy.callback_groups import ReentrantCallbackGroup, MutuallyExclusiveCallbackGroup
from sensor_msgs.msg import JointState
from std_srvs.srv import SetBool
from moonbot_custom_interfaces.msg import Detection

import dynamixel_sdk as dxl



class State_subscriber(Node):

    def __init__(self):
        super().__init__('state_subscriber')

        ##### Callbackgroup #####
        self.group = ReentrantCallbackGroup()
        self.RF_timer_group = MutuallyExclusiveCallbackGroup()
        self.LF_timer_group = MutuallyExclusiveCallbackGroup()
        self.LR_timer_group = MutuallyExclusiveCallbackGroup()
        self.RR_timer_group = MutuallyExclusiveCallbackGroup()


        ##### TIMER #####
        self.timer_period = 1
        self.RF_intualizer = self.create_timer(self.timer_period, self.RF_initualizer_callback, callback_group=self.RF_timer_group)
        self.LF_intualizer = self.create_timer(self.timer_period, self.LF_initualizer_callback, callback_group=self.LF_timer_group)
        self.LR_intualizer = self.create_timer(self.timer_period, self.LR_initualizer_callback, callback_group=self.LR_timer_group)
        self.RR_intualizer = self.create_timer(self.timer_period, self.RR_initualizer_callback, callback_group=self.RR_timer_group)
        self.status_timer = self.create_timer(2, self.status_callback, callback_group=self.group)
        ##### TIMER #####

        ##### PUBLISHER #####
        self.connection_status_pub = self.create_publisher(Detection, "moonbot_detection", 10)
        ##### PUBLISHER #####

        ##### Subcriber ##### 
        self.RF_state = self.create_subscription(
            JointState,
            '/RF/joint_states',
            self.RF_state_callback,
            10, 
            callback_group=self.group)
        self.RF_state

        self.LF_state = self.create_subscription(
            JointState,
            '/LF/joint_states',
            self.LF_state_callback,
            10,
            callback_group=self.group)
        self.LF_state

        self.LR_state = self.create_subscription(
            JointState,
            '/LR/joint_states',
            self.LR_state_callback,
            10,
            callback_group=self.group)
        self.LR_state

        self.RR_state = self.create_subscription(
            JointState,
            '/RR/joint_states',
            self.RR_state_callback,
            10,
            callback_group=self.group)
        self.RR_state
        ##### Subcriber #####

        ##### Service Client #####
        self.RF_ConnectionClient = self.create_client(SetBool, 'RF_trigger')
        while not self.RF_ConnectionClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RF service not available, waiting again...')
        self.RFreq = SetBool.Request()

        self.LF_ConnectionClient = self.create_client(SetBool, 'LF_trigger')
        while not self.LF_ConnectionClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LF service not available, waiting again...')
        self.LFreq = SetBool.Request()

        self.LR_ConnectionClient = self.create_client(SetBool, 'LR_trigger')
        while not self.LR_ConnectionClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('LR service not available, waiting again...')
        self.LRreq = SetBool.Request()

        self.RR_ConnectionClient = self.create_client(SetBool, 'RR_trigger')
        while not self.RR_ConnectionClient.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('RR service not available, waiting again...')
        self.RRreq = SetBool.Request()
        ##### Service Client #####


        self.RF_detected = False
        self.RF_connected = False

        self.LF_detected = False
        self.LF_connected = False
        
        self.LR_detected = False
        self.LR_connected = False

        self.RR_detected = False
        self.RR_connected = False
        
        self.RF_state = 0.0
        self.LF_state = 0.0
        self.LR_state = 0.0
        self.RR_state = 0.0


    def RF_send_request(self, request):
        self.RFreq.data = request
        self.future = self.RF_ConnectionClient.call_async(self.RFreq)

    def LF_send_request(self, request):
        self.LFreq.data = request
        self.future = self.LF_ConnectionClient.call_async(self.LFreq)

    def LR_send_request(self, request):
        self.LRreq.data = request
        self.future = self.LR_ConnectionClient.call_async(self.LRreq)

    def RR_send_request(self, request):
        self.RRreq.data = request
        self.future = self.RR_ConnectionClient.call_async(self.RRreq)

    
    def status_callback(self):
        msg = Detection()
        msg.data = [self.RF_connected, self.LF_connected, self.LR_connected, self.RR_connected]
        self.connection_status_pub.publish(msg)
        self.get_logger().info(f"Status: {msg.data}")


    def RF_initualizer_callback(self):
        # self.get_logger().info(f'NOW RF STATE: {self.RF_state}')

        if self.RF_detected == False:
        
            if self.dxl_is_detected("/dev/moonbot_RF", 4000000, 1):
                self.RF_send_request(True)
                self.get_logger().info('RF is detected!')
                time.sleep(5)
                self.RF_detected = True
                self.RF_connected = True
            else:
                self.get_logger().warn('No RF tate detected!')

        elif self.RF_detected:
            if self.RF_connected == True:
                RF_state = float(self.RF_state)

                if abs(RF_state) > 3.14:
                    self.RF_send_request(False)
                    self.get_logger().info(f"RF LOSE!")
                    self.RF_connected = False
                    self.RF_detected = False  
                else:
                    self.get_logger().info(f"RF CONNECTED!")
                    self.RF_detected = True


    def LF_initualizer_callback(self):
        # self.get_logger().info(f'NOW LF STATE: {self.LF_state}')

        if self.LF_detected == False:
        
            if self.dxl_is_detected("/dev/moonbot_LF", 4000000, 1):
                self.LF_send_request(True)
                self.get_logger().info('LF is detected!')
                time.sleep(5)
                self.LF_detected = True
                self.LF_connected = True
            else:
                self.get_logger().warn('No LF state detected!')

        elif self.LF_detected:
            if self.LF_connected == True:
                LF_state = float(self.LF_state)

                if abs(LF_state) > 3.14:
                    self.LF_send_request(False)
                    self.get_logger().info(f"LF LOSE!")
                    self.LF_connected = False
                    self.LF_detected = False  
                else:
                    self.get_logger().info(f"LF CONNECTED!")
                    self.LF_detected = True
    

    def LR_initualizer_callback(self):
        # self.get_logger().info(f'NOW LR STATE: {self.LR_state}')

        if self.LR_detected == False:
        
            if self.dxl_is_detected("/dev/moonbot_LR", 4000000, 1):
                self.LR_send_request(True)
                self.get_logger().info('LR is detected!')
                time.sleep(5)
                self.LR_detected = True
                self.LR_connected = True
            else:
                self.get_logger().warn('No LR state detected!')

        elif self.LR_detected:
            if self.LR_connected == True:
                LR_state = float(self.LR_state)

                if abs(LR_state) > 3.14:
                    # time.sleep(1)
                    self.LR_send_request(False)
                    self.get_logger().info(f"LR LOSE!")
                    self.LR_connected = False
                    self.LR_detected = False  
                else:
                    self.get_logger().info(f"LR CONNECTED!")
                    self.LR_detected = True


    def RR_initualizer_callback(self):
        # self.get_logger().info(f'NOW RR STATE: {self.RR_state}')

        if self.RR_detected == False:
        
            if self.dxl_is_detected("/dev/moonbot_RR", 4000000, 1):
                self.RR_send_request(True)
                self.get_logger().info('RR is detected!')
                time.sleep(5)
                self.RR_detected = True
                self.RR_connected = True
            else:
                self.get_logger().warn('No RR state detected!')

        elif self.RR_detected:
            if self.RR_connected == True:
                RR_state = float(self.RR_state)

                if abs(RR_state) > 3.14:
                    # time.sleep(1)
                    self.RR_send_request(False)
                    self.get_logger().info(f"RR LOSE!")
                    self.RR_connected = False
                    self.RR_detected = False  
                else:
                    self.get_logger().info(f"RR CONNECTED!")
                    self.RR_detected = True



    def dxl_is_detected(self, port_name, baud_rate, motor_id):
        # Initialize the Dynamixel SDK
        dynamixel_lib = dxl

        # Set the port name, baudrate, and motor ID
        port_handler = dynamixel_lib.PortHandler(port_name)
        packet_handler = dynamixel_lib.PacketHandler(2.0)
        
        # Open the port
        if port_handler.openPort():
            self.get_logger().info(f"Port {port_name} opened successfully.")
        else:
            self.get_logger().warn(f"Failed to open port {port_name}.")
            return False

        # Set the baudrate
        if port_handler.setBaudRate(baud_rate):
            self.get_logger().info(f"Baudrate set to {baud_rate}.")
        else:
            self.get_logger().warn(f"Failed to set baudrate to {baud_rate}.")
            return False

        # Ping the Dynamixel motor
        dxl_model_number, dxl_comm_result, dxl_error = packet_handler.ping(port_handler, motor_id)

        if dxl_comm_result == dynamixel_lib.COMM_SUCCESS and dxl_model_number != dynamixel_lib.ERRBIT_VOLTAGE:
            self.get_logger().info(f"{port_name} is detected.")
            return True
        else:
            self.get_logger().warn(f"Failed to detect {port_name}.")
            return False



    def RF_state_callback(self, state):
        self.RF_state = state.position[0]

    def LF_state_callback(self, state):
        self.LF_state = state.position[0]

    def LR_state_callback(self, state):
        self.LR_state = state.position[0]

    def RR_state_callback(self, state):
        self.RR_state = state.position[0]
        


def main(args=None):
    rclpy.init(args=None)
    try:
        state_subscriber = State_subscriber()

        executor = MultiThreadedExecutor()
        executor.add_node(state_subscriber)

        try:
            executor.spin()
        finally:
            executor.shutdown()
    finally:  
        state_subscriber.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()

