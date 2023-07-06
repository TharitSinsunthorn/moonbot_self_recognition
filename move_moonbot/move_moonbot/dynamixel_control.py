import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
from rclpy.executors import MultiThreadedExecutor
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup
# from std_msgs.msg import Float64
from moonbot_custom_interfaces.msg import SetPosition
from moonbot_custom_interfaces.srv import GetPosition
import sys, tty, termios
import threading


# For Linux OS

'''
This exception rule is because when using launch file, the node was dieing 
because of some termios error.
'''
try:
    fd = sys.stdin.fileno()
    old_settings = termios.tcgetattr(fd)
except:
    print("Some Linux OS setting might not be working")
def getch():
    try:
        tty.setraw(sys.stdin.fileno())
        ch = sys.stdin.read(1)
    finally:
        termios.tcsetattr(fd, termios.TCSADRAIN, old_settings)
    return ch

#********* DYNAMIXEL Model definition *********
#***** (Use only one definition at a time) *****
MY_DXL = 'X_SERIES'       # X330 (5.0 V recommended), X430, X540, 2X430
# MY_DXL = 'MX_SERIES'    # MX series with 2.0 firmware update.
# MY_DXL = 'PRO_SERIES'   # H54, H42, M54, M42, L54, L42
# MY_DXL = 'PRO_A_SERIES' # PRO series with (A) firmware update.
# MY_DXL = 'P_SERIES'     # PH54, PH42, PM54
# MY_DXL = 'XL320'        # [WARNING] Operating Voltage : 7.4V

# Dynamixel Control table addresses
if MY_DXL == 'X_SERIES' or MY_DXL == 'MX_SERIES':
    ADDR_TORQUE_ENABLE = 64
    ADDR_GOAL_POSITION = 116
    ADDR_PRESENT_POSITION = 132
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 4095      # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600     ## determines serial communication speed between a controller and DYNAMIXEL.
elif MY_DXL == 'PRO_SERIES':
    ADDR_TORQUE_ENABLE          = 562       # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 596
    ADDR_PRESENT_POSITION       = 611
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'P_SERIES' or MY_DXL == 'PRO_A_SERIES':
    ADDR_TORQUE_ENABLE          = 512        # Control table address is different in DYNAMIXEL model
    ADDR_GOAL_POSITION          = 564
    ADDR_PRESENT_POSITION       = 580
    DXL_MINIMUM_POSITION_VALUE  = -150000   # Refer to the Minimum Position Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 150000    # Refer to the Maximum Position Limit of product eManual
    BAUDRATE                    = 57600
elif MY_DXL == 'XL320':
    ADDR_TORQUE_ENABLE          = 24
    ADDR_GOAL_POSITION          = 30
    ADDR_PRESENT_POSITION       = 37
    DXL_MINIMUM_POSITION_VALUE  = 0         # Refer to the CW Angle Limit of product eManual
    DXL_MAXIMUM_POSITION_VALUE  = 1023      # Refer to the CCW Angle Limit of product eManual
    BAUDRATE                    = 1000000   # Default Baudrate of XL-320 is 1Mbps

# Default setting
PROTOCOL_VERSION = 2.0
TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold determines whether the DYNAMIXEL is in motion or not. 

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

DEVICES = ['/dev/ttyUSB0', '/dev/ttyUSB1']

class DynamixelControl(Node):
    def __init__(self):
        super().__init__('dynamixel_control')
        self.get_logger().info(f'Launched Dynamixel Control Node')
        self.portHandler = [PortHandler(DEVICES[i]) for i in range(2)]
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open the port
        for i in range(2):
            if self.portHandler[i].openPort():
                self.get_logger().info(f'Succeeded to open the port at {DEVICES[i]}')
            else:
                self.get_logger().info(f'Failed to open the port at {DEVICES[i]}')
                self.get_logger().info("Press any key to terminate...")
                getch()
                quit()
                return

        # Set port baudrate
        for i in range(2):
            if self.portHandler[i].setBaudRate(BAUDRATE):
                self.get_logger().info(f'Succeeded to set the baudrate at port {i + 1}')
            else:
                self.get_logger().info(f'Failed to change the baudrate at port {i + 1}')
                self.get_logger().info("Press any key to terminate...")
                getch()
                quit()
                return

        # Set QOS
        qos_depth = 0
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL1OV = rclpy.qos.QoSProfile(
            depth=qos_depth,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        self.timer_cb_group = MutuallyExclusiveCallbackGroup()
        self.subs_cb_group = MutuallyExclusiveCallbackGroup()

        # Subscribe to the topic to receive the desired position
        self.set_position_subscriber = self.create_subscription(
            SetPosition,
            f'set_position',
            self.add_subs_clbk,
            QOS_RKL1OV)
        self.set_position_subscriber  # prevent unused variable warning

        self.get_position_server_ = self.create_service(
            GetPosition,
            f'get_position',
            self.add_service_clbk)
        
        self.JOINT_STATUS = [True] * 12
        self.LIMB_STATUS = [True, True, True, True]
        for i in range(2):
            self.set_torque_enable(TORQUE_ENABLE, BROADCAST_ID, self.portHandler[i])
        self.timer_update_limb_status = self.create_timer(0.5, self.add_time_clbk)
        # self.__del__()
        self.subcriber_clbk_list = []
        self.timer_clbk_list = []
        # self.spin()

    def add_time_clbk(self, *args):
        self.get_logger().info("Adding callback to timer Function")
        self.timer_clbk_list.append((self.update_limb_status, args))
    
    def add_subs_clbk(self, *args):
        self.subcriber_clbk_list.append((self.set_position_callback, args))

    def add_service_clbk(self, *args):
        self.subcriber_clbk_list.append((self.get_present_position_callback, args))
        return GetPosition.Response()


    def update_limb_status(self):
        dxl_devices = []
        for i in range(2):
            dxl_data_list, _dxl_comm_result = self.packetHandler.broadcastPing(self.portHandler[i])
            if _dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(_dxl_comm_result))
            dxl_devices += dxl_data_list
        self.JOINT_STATUS = [False] * 12
        for id in dxl_devices:
            self.JOINT_STATUS[id - 1] = True

        for i in range(4):
            if self.JOINT_STATUS[i * 3] or self.JOINT_STATUS[i * 3 + 1] or self.JOINT_STATUS[i * 3 + 2]:
                if self.LIMB_STATUS[i] == False:
                    self.LIMB_STATUS[i] = True
                    for j in range(3):
                        id = i *3 + j + 1
                        portHandler_ID = (id - 1)//6
                        # self.packetHandler.reboot(self.portHandler[portHandler_ID], id)
                        self.set_torque_enable(TORQUE_ENABLE, id, self.portHandler[portHandler_ID])
                    self.get_logger().info(f"Limb {i + 1} attached !!")
            else:
                if self.LIMB_STATUS[i] == True:
                    self.LIMB_STATUS[i] = False
                    self.get_logger().info(f"Limb {i + 1} detached !!")

    def set_torque_enable(self, enable, id, port):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            port, id, ADDR_TORQUE_ENABLE, enable)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info(self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info(self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.get_logger().info(f"Torque Enabled for Sevo ID {id}")


    def set_position_callback(self, msg):
        # dxl_error = 0

        # Position Value of X series is 4 byte data.
        # For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
        goal_position = int(msg.position)  # Convert int32 to int
        id = int(msg.id)
        portHandler_ID = (id - 1)//6

        # Write Goal Position (length : 4 bytes)
        # When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler[portHandler_ID],
            int(msg.id),
            ADDR_GOAL_POSITION,
            goal_position)

        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info(self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info(self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.get_logger().info('Set [ID: %d] [Goal Position: %d]' % (msg.id, msg.position))
        
    def get_present_position_callback(self, request, response):
        # Read Present Position (length : 4 bytes) and Convert uint32 -> int32
        id = request.id
        portHandler_ID = (id - 1)//6
        present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler[portHandler_ID],
            id,
            ADDR_PRESENT_POSITION,
        )
    
        response.position = int(present_position)

        self.get_logger().info("Get [ID: %d] [Present Position: %d]" % (request.id, response.position))
       
        return response

        # self.logger().info(f"Get [ID: {request.id}] [Present Position: {response.position}]")
    
    
    def __del__(self):
        # Disable Dynamixel Torque

        # Close the port
        for i in range(2):
            self.set_torque_enable(TORQUE_DISABLE, BROADCAST_ID, self.portHandler[i])
            self.portHandler[i].closePort()
            self.get_logger().info('Port closed')


def main(args=None):
    rclpy.init(args=args)
    dynamixel_control = DynamixelControl()
    while rclpy.ok():
        if len(dynamixel_control.subcriber_clbk_list) !=0:
            callback, args = dynamixel_control.subcriber_clbk_list.pop(0)
            callback(*args)
        elif len(dynamixel_control.timer_clbk_list) != 0:
            callback, args = dynamixel_control.timer_clbk_list.pop(0)
            callback(*args)
        rclpy.spin_once(dynamixel_control, timeout_sec=0.1)
    # rclpy.spin(dynamixel_control)
    # executor = MultiThreadedExecutor()
    # executor.add_node(dynamixel_control)

    # executor_thread = threading.Thread(target = executor.spin)
    # node_spin = threading.Thread(target = dynamixel_control.spin)
    # executor_thread.start()
    # node_spin.start()

    dynamixel_control.destroy_node()
    # executor.shutdown()
    # executor_thread.join()
    # node_spin.join()
    rclpy.shutdown()

    dynamixel_control.__del__()

if __name__ == '__main__':
    main()