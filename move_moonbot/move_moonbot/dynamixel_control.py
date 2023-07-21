import rclpy
from rclpy.node import Node
from dynamixel_sdk import *
# from std_msgs.msg import Float64
from moonbot_custom_interfaces.msg import SetPosition
from moonbot_custom_interfaces.msg import JointAngles
from moonbot_custom_interfaces.srv import GetPosition
import sys, tty, termios
from move_moonbot.utilities.utilities import changedeg2value, changevalue2deg
import move_moonbot.utilities.params as params
from functools import partial
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
# PROFILE_ACCELERATION = 1
TORQUE_ENABLE = 1  # Value for enabling the torque
TORQUE_DISABLE = 0  # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20    # Dynamixel moving status threshold determines whether the DYNAMIXEL is in motion or not. 
# Data Byte Length
LEN_PRO_GOAL_POSITION      = 4
LEN_PRO_PRESENT_POSITION   = 4

index = 0
dxl_goal_position = [DXL_MINIMUM_POSITION_VALUE, DXL_MAXIMUM_POSITION_VALUE]         # Goal position

DEVICES = ['/dev/ttyUSB0', '/dev/ttyUSB1']

class DynamixelControl(Node):
    def __init__(self):
        super().__init__('dynamixel_control')
        self.declare_parameter('dynamixel_num', None)
        self.dynamixel_num = self.get_parameter('dynamixel_num').get_parameter_value().integer_value
        self.get_logger().info(f'Launched Dynamixel Control Node {self.dynamixel_num}')
        self.portHandler = PortHandler(DEVICES[self.dynamixel_num - 1])
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        if self.portHandler.openPort():
            self.get_logger().info(f'Succeeded to open the port at {DEVICES[self.dynamixel_num - 1]}')
        else:
            self.get_logger().info(f'Failed to open the port at {DEVICES[self.dynamixel_num - 1]}')
            self.get_logger().info("Press any key to terminate...")
            getch()
            quit()
            return

        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            self.get_logger().info(f'Succeeded to set the baudrate at port {self.dynamixel_num}')
        else:
            self.get_logger().info(f'Failed to change the baudrate at port {self.dynamixel_num}')
            self.get_logger().info("Press any key to terminate...")
            getch()
            quit()
            return

        # Enable Dynamixel Torque


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
        self.set_joint_angle_2 = self.create_subscription(JointAngles, f'target_joint_angles_l{(self.dynamixel_num - 1) * 2 + 1}', lambda msg: self.set_jointAngle_clbk(msg,  (self.dynamixel_num - 1) * 2 + 1), 10)
        self.set_joint_angle_2 = self.create_subscription(JointAngles, f'target_joint_angles_l{(self.dynamixel_num - 1) * 2 + 2}', lambda msg: self.set_jointAngle_clbk(msg,  (self.dynamixel_num - 1) * 2 + 2), 10)

        self.groupSyncWrite = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

        self.get_position_server_ = self.create_service(
            GetPosition,
            f'get_position_d{self.dynamixel_num}',
            self.get_present_position_callback
        )
        self.LIMB_STATUS = [True, True]
        self.set_torque_enable(TORQUE_ENABLE, BROADCAST_ID, self.portHandler)
        # self.timer_update_limb_status = self.create_timer(1, self.update_limb_status)
        self.i = 0
        # self.__del__()

    def set_jointAngle_clbk(self, msg, leg_num):
        self.servo_id = params.servo_id[str(leg_num)]
        position2 = changedeg2value(msg.joint2, self.servo_id[1][1])
        position1 = changedeg2value(msg.joint1, self.servo_id[0][1])
        position3 = changedeg2value(msg.joint3, self.servo_id[2][1])
        self.groupSyncWrite.addParam(self.servo_id[1][0], [DXL_LOBYTE(DXL_LOWORD(position2)), DXL_HIBYTE(DXL_LOWORD(position2)), DXL_LOBYTE(DXL_HIWORD(position2)), DXL_HIBYTE(DXL_HIWORD(position2))])
        self.groupSyncWrite.addParam(self.servo_id[2][0], [DXL_LOBYTE(DXL_LOWORD(position3)), DXL_HIBYTE(DXL_LOWORD(position3)), DXL_LOBYTE(DXL_HIWORD(position3)), DXL_HIBYTE(DXL_HIWORD(position3))])
        self.groupSyncWrite.addParam(self.servo_id[0][0], [DXL_LOBYTE(DXL_LOWORD(position1)), DXL_HIBYTE(DXL_LOWORD(position1)), DXL_LOBYTE(DXL_HIWORD(position1)), DXL_HIBYTE(DXL_HIWORD(position1))])

        dxl_comm_result  = self.groupSyncWrite.txPacket()
        if dxl_comm_result != COMM_SUCCESS:
            self.get_logger().info(self.packetHandler.getTxRxResult(dxl_comm_result))
        else:
            self.get_logger().info(f"Set [ID: {self.servo_id[0][0]}] [Goal Position: {position1}], [ID: {self.servo_id[1][0]}] [Goal Position: {position2}], [ID: {self.servo_id[2][0]}] [Goal Position: {position3}]")
        self.groupSyncWrite.clearParam()

    def update_limb_status_temp(self):
        self.get_logger().info(f"Well me is running kido!! {self.i}")
        self.i+=1

    def update_limb_status(self):
        for i in range(2):
            limb_id = 6 *(self.dynamixel_num - 1) + 3 * i + 1
            dxl_model_number, dxl_comm_result, dxl_error = self.packetHandler.ping(self.portHandler, limb_id)
            # self.get_logger().info(f'{dxl_comm_result}')
            if dxl_comm_result != COMM_SUCCESS or dxl_error !=0:
                if self.LIMB_STATUS[i] == True:
                    self.LIMB_STATUS[i] = False
                    self.get_logger().info(f"Limb {2 * (self.dynamixel_num - 1) + i + 1} detached !!")
            else:
                if self.LIMB_STATUS[i] == False:
                    self.LIMB_STATUS[i] = True
                    for j in range(3):
                        id = limb_id + j
                        self.set_torque_enable(TORQUE_ENABLE, id, self.portHandler)
                    self.get_logger().info(f"Limb {2 * (self.dynamixel_num - 1) + i + 1} attached !!")

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

        # Write Goal Position (length : 4 bytes)
        # When writing 2 byte data to AX / MX(1.0), use write2ByteTxRx() instead.
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler,
            id,
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
        present_position, dxl_comm_result, dxl_error = self.packetHandler.read4ByteTxRx(
            self.portHandler,
            id,
            ADDR_PRESENT_POSITION,
        )
    
        response.position = int(present_position)

        self.get_logger().info("Get [ID: %d] [Present Position: %d]" % (request.id, response.position))
       
        return response

        # self.logger().info(f"Get [ID: {request.id}] [Present Position: {response.position}]")
    
    
    def __del__(self):
        # Disable Dynamixel Torque
        self.set_torque_enable(TORQUE_DISABLE, BROADCAST_ID, self.portHandler)

        # Close the port
        self.portHandler.closePort()
        self.get_logger().info('Port closed')


def main(args=None):
    rclpy.init(args=args)

    dynamixel_control = DynamixelControl()
    rate = dynamixel_control.create_rate(2)
    
    while rclpy.ok():
        rclpy.spin_once(dynamixel_control, timeout_sec=0.5)
        dynamixel_control.update_limb_status()
        # time.sleep(0.2)
    # rclpy.spin(dynamixel_control)

    dynamixel_control.destroy_node()
    rclpy.shutdown()

    dynamixel_control.__del__()

if __name__ == '__main__':
    main()