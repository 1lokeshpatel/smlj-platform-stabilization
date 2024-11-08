# Motor Control Application Code

# TODO: Test with our motors 
# TODO: Figure out needed baudrate

import os, sys, ctypes, time
os.sys.path.append('../dynamixel_functions_py')             # Path setting
import dynamixel_functions as dynamixel                     # Uses DYNAMIXEL SDK library

# Control table address - specifically for XM430-W210
ADDR_PRO_TORQUE_ENABLE      = 64
ADDR_PRO_GOAL_POSITION      = 116
ADDR_PRO_PRESENT_POSITION   = 132

# Data Byte Length
LEN_PRO_GOAL_POSITION       = 4
LEN_PRO_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2                             

# Default setting
DXL1_ID                     = 1                             # Dynamixel ID: 1
DXL2_ID                     = 2                             # Dynamixel ID: 2
DXL3_ID                     = 3                             # Dynamixel ID: 3
BAUDRATE                    = 1000000
DEVICENAME                  = "/dev/ttyUSB0"                # TODO: Check which port is being used on RPI

TORQUE_ENABLE               = 1                             # Value for enabling the torque
TORQUE_DISABLE              = 0                             # Value for disabling the torque
DXL_MOVING_STATUS_THRESHOLD = 20                            # Dynamixel moving status threshold

COMM_SUCCESS                = 0                             # Communication Success result value
COMM_TX_FAIL                = -1001                         # Communication Tx Failed

class MotorControl:
    def __init__(self):
        # Initialize PortHandler Structs
        # Set the port path
        # Get methods and members of PortHandlerLinux or PortHandlerWindows
        self.port_num = dynamixel.portHandler(DEVICENAME)

        # Initialize PacketHandler Structs
        dynamixel.packetHandler()

        # Initialize Groupsyncwrite instance
        self.groupwrite_num = dynamixel.groupSyncWrite(self.port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

        # Initialize Groupsyncread Structs for Present Position
        self.groupread_num = dynamixel.groupSyncRead(self.port_num, PROTOCOL_VERSION, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        self.dxl_comm_result = COMM_TX_FAIL                              # Communication result
        self.dxl_addparam_result = 0                                     # AddParam result

    def setup(self):
        # Open port
        if dynamixel.openPort(self.port_num):
            print("Succeeded to open the port!")
        else:
            print("Failed to open port")
            return False
        # Set port baudrate
        if dynamixel.setBaudRate(self.port_num, BAUDRATE):
            print("Succeeded to change the baudrate!")
        else:
            print("Failed to set baudrate")
            return False
        
        self.enable_torque(DXL1_ID)
        self.enable_torque(DXL2_ID)
        self.enable_torque(DXL3_ID)

        return True

    def move_motor(self, motorPos1, motorPos2, motorPos3):
        # Present positions
        dxl1_present_position = 0                                   
        dxl2_present_position = 0
        dxl3_present_position = 0

        # [motorPos1, motorPos2, motorPos3]
        dxl_goal_position = [motorPos1, motorPos2, motorPos3]

        timeout = 5  # Timeout in seconds
        start_time = time.time()

        while 1:

            # Add Dynamixel#1 goal position value to the Syncwrite storage
            self.dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(self.groupwrite_num, DXL1_ID, dxl_goal_position[DXL1_ID-1], LEN_PRO_GOAL_POSITION)).value
            print(self.dxl_addparam_result)
            if self.dxl_addparam_result != 1:
                print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
                quit()

            # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
            self.dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(self.groupwrite_num, DXL2_ID, dxl_goal_position[DXL2_ID-1], LEN_PRO_GOAL_POSITION)).value
            if self.dxl_addparam_result != 1:
                print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
                quit()
            
            # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
            self.dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(self.groupwrite_num, DXL3_ID, dxl_goal_position[DXL3_ID-1], LEN_PRO_GOAL_POSITION)).value
            if self.dxl_addparam_result != 1:
                print("[ID:%03d] groupSyncWrite addparam failed" % (DXL3_ID))
                quit()

            # Syncwrite goal position
            dynamixel.groupSyncWriteTxPacket(self.groupwrite_num)
            if dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION))

            # Clear syncwrite parameter storage
            dynamixel.groupSyncWriteClearParam(self.groupwrite_num)

            while 1:
                # Syncread present position
                dynamixel.groupSyncReadTxRxPacket(self.groupread_num)
                if dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
                    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION))

                # Check if groupsyncread data of Dynamixel#1 is available
                dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(self.groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)).value
                if dxl_getdata_result != 1:
                    print("[ID:%03d] groupSyncRead getdata failed" % (DXL1_ID))
                    quit()

                # Check if groupsyncread data of Dynamixel#2 is available
                dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(self.groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)).value
                if dxl_getdata_result != 1:
                    print("[ID:%03d] groupSyncRead getdata failed" % (DXL2_ID))
                    quit()

                # Check if groupsyncread data of Dynamixel#3 is available
                dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(self.groupread_num, DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)).value
                if dxl_getdata_result != 1:
                    print("[ID:%03d] groupSyncRead getdata failed" % (DXL3_ID))
                    quit()

                # Get Dynamixel#1 present position value
                dxl1_present_position = dynamixel.groupSyncReadGetData(self.groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

                # Get Dynamixel#2 present position value
                dxl2_present_position = dynamixel.groupSyncReadGetData(self.groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

                # Get Dynamixel#3 present position value
                dxl3_present_position = dynamixel.groupSyncReadGetData(self.groupread_num, DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

                print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n" % 
                    (DXL1_ID, dxl_goal_position[DXL1_ID-1], dxl1_present_position, DXL2_ID, dxl_goal_position[DXL2_ID-1], dxl2_present_position, DXL3_ID, dxl_goal_position[DXL3_ID-1], dxl3_present_position))

                if not ((abs(dxl_goal_position[DXL1_ID-1] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) or (abs(dxl_goal_position[DXL2_ID-1] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD) or 
                        (abs(dxl_goal_position[DXL3_ID-1] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                    break

                # Check for timeout - break loop if timeout is reached
                if time.time() - start_time > timeout:
                    print("Timeout reached. Exiting loop.")
                    break

        print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n" % 
                    (DXL1_ID, dxl_goal_position[DXL1_ID-1], dxl1_present_position, DXL2_ID, dxl_goal_position[DXL2_ID-1], dxl2_present_position, DXL3_ID, dxl_goal_position[DXL3_ID-1], dxl3_present_position))

    def shutdown(self):
        self.disable_torque(DXL1_ID)
        self.disable_torque(DXL2_ID)
        self.disable_torque(DXL3_ID)

        # Close port
        dynamixel.closePort(self.port_num)

    def enable_torque(self, motor_id):
        # Enable Dynamixel Torque
        dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
        if dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION))
            return False
        elif dynamixel.getLastRxPacketError(self.port_num, PROTOCOL_VERSION) != 0:
            dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, PROTOCOL_VERSION))
            return False
        else:
            print("Dynamixel#%d has been successfully connected" % motor_id)

        # Add parameter storage for Dynamixel present position value
        self.dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(self.groupread_num, motor_id)).value
        if self.dxl_addparam_result != 1:
            print("[ID:%03d] groupSyncRead addparam failed" % motor_id)
            quit()
    
    def disable_torque(self, motor_id):
        # Disable Dynamixel Torque
        dynamixel.write1ByteTxRx(self.port_num, PROTOCOL_VERSION, motor_id, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
        if dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(self.port_num, PROTOCOL_VERSION))
        elif dynamixel.getLastRxPacketError(self.port_num, PROTOCOL_VERSION) != 0:
            dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(self.port_num, PROTOCOL_VERSION))

# Convert angle to Dynamixel position (0-4095 range)
def angle_to_position(angle):
    return int((angle / 360) * 4095)
