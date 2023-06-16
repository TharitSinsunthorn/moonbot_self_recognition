import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
# from std_msgs.msg import Float64
from dynamixel_custom_interfaces.msg import SetPosition
from dynamixel_custom_interfaces.srv import GetPosition
import sys, tty, termios

# For Linux OS
fd = sys.stdin.fileno()
old_settings = termios.tcgetattr(fd)
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
DXL_ID = 1  # Dynamixel ID
DEVICENAME = '/dev/ttyUSB0'  # Check which port is being used on your controller
TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold determines whether the DYNAMIXEL is in motion or not. 

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

class DynamixelControl(Node):
    def __init__(self):
        super().__init__('dynamixel_control')

        self.get_logger().info('dynamixel_control_node')

        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Open the port
        if self.portHandler.openPort():
            self.get_logger().info('Succeeded to open the port')
        else:
            self.get_logger().info('Failed to open the port')
            self.get_logger().info("Press any key to terminate...")
            getch()
            quit()
            return

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info('Succeeded to set the baudrate')
        else:
            self.get_logger().info('Failed to change the baudrate')
            self.get_logger().info("Press any key to terminate...")
            getch()
            quit()
            return

        # Enable Dynamixel Torque
        self.set_torque_enable(TORQUE_ENABLE)


        # Set QOS
        qos_depth = 0
        self.declare_parameter('qos_depth', 10)
        qos_depth = self.get_parameter('qos_depth').value

        QOS_RKL1OV = rclpy.qos.QoSProfile(
            depth=qos_depth,
            reliability=rclpy.qos.QoSReliabilityPolicy.RELIABLE,
            durability=rclpy.qos.QoSDurabilityPolicy.VOLATILE
        )

        # Subscribe to the topic to receive the desired position
        self.set_position_subscriber = self.create_subscription(
            SetPosition,
            'set_position',
            self.set_position_callback,
            QOS_RKL1OV)
        self.set_position_subscriber  # prevent unused variable warning

        self.get_position_server_ = self.create_service(
            GetPosition,
            'get_position',
            self.get_present_position_callback
        )

        # self.__del__()

    def set_torque_enable(self, enable):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, BROADCAST_ID, ADDR_TORQUE_ENABLE, enable)
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info(self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            self.get_logger().info(self.packetHandler.getRxPacketError(dxl_error))
        else:
            self.get_logger().info("Dynamixel has been successfully connected")

    # def set_goal_position(self, id, position):
    #     dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
    #         self.portHandler, DXL_ID, ADDR_GOAL_POSITION, int(position))
    #     if dxl_comm_result != COMM_SUCCESS:
    #         self.get_logger().info(self.packetHandler.getTxRxResult(dxl_comm_result))
    #     elif dxl_error != 0:
    #         self.get_logger().info(self.packetHandler.getRxPacketError(dxl_error))
    #     else:
    #         self.get_logger().info('Set [ID: %d] [Goal Position: %d]', id, position)


    def set_position_callback(self, msg):
        # dxl_error = 0

        # Position Value of X series is 4 byte data.
        # For AX & MX(1.0) use 2 byte data(uint16_t) for the Position Value.
        goal_position = int(msg.position)  # Convert int32 to int

        # Write Goal Position (length : 4 bytes)
        # When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler,
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
        present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler,
            request.id,
            ADDR_PRESENT_POSITION,
        )

    
        response.position = int(present_position)

        self.get_logger().info("Get [ID: %d] [Present Position: %d]" % (request.id, response.position))
       
        return response

        # self.logger().info(f"Get [ID: {request.id}] [Present Position: {response.position}]")
    
    
    def __del__(self):
        # Disable Dynamixel Torque
        self.set_torque_enable(TORQUE_DISABLE)

        # Close the port
        self.portHandler.closePort()
        self.get_logger().info('Port closed')


def main(args=None):
    rclpy.init(args=args)

    dynamixel_control = DynamixelControl()

    rclpy.spin(dynamixel_control)

    dynamixel_control.destroy_node()
    rclpy.shutdown()

    dynamixel_control.__del__()

if __name__ == '__main__':
    main()
