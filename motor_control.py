# Motor Control Application Code

# TODO: Test with our motors 
# TODO: Figure out needed baudrate

import time
from dynamixel_sdk import *

# Control table address - specifically for XM430-W210
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132

# Data Byte Length
LEN_GOAL_POSITION       = 4
LEN_PRESENT_POSITION    = 4

# Protocol version
PROTOCOL_VERSION            = 2                             

# Default setting
DXL1_ID                     = 1                             # Dynamixel ID: 1
DXL2_ID                     = 2                             # Dynamixel ID: 2
DXL3_ID                     = 3                             # Dynamixel ID: 3
BAUDRATE                    = 57600
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
        self.portHandler = PortHandler(DEVICENAME)

        # Initialize PacketHandler Structs
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)

        # Initialize Groupsyncwrite instance
        self.groupwrite_num = GroupSyncWrite(self.portHandler, self.packetHandler, ADDR_GOAL_POSITION, LEN_GOAL_POSITION)

        # Initialize Groupsyncread Structs for Present Position
        self.groupread_num = GroupSyncRead(self.portHandler, self.packetHandler, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        self.dxl_comm_result = COMM_TX_FAIL                              # Communication result
        self.dxl_addparam_result = 0                                     # AddParam result

    def setup(self):
        # Open port
        if self.portHandler.openPort():
            print("Succeeded to open the port!")
        else:
            print("Failed to open port")
            return False
        # Set port baudrate
        if self.portHandler.setBaudRate(BAUDRATE):
            print("Succeeded to change the baudrate!")
        else:
            print("Failed to set baudrate")
            return False
        
        self.enable_torque(DXL1_ID)
        self.enable_torque(DXL2_ID)
        # self.enable_torque(DXL3_ID)

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

            # Add goal position values to the SyncWrite parameter storage for each motor
            for i, dxl_id in enumerate([DXL1_ID, DXL2_ID]):
                # Extract goal position for each motor and pack it into 4 bytes (int32)
                goal_position = dxl_goal_position[i]
                param_goal_position = [
                    DXL_LOBYTE(DXL_LOWORD(goal_position)), 
                    DXL_HIBYTE(DXL_LOWORD(goal_position)), 
                    DXL_LOBYTE(DXL_HIWORD(goal_position)), 
                    DXL_HIBYTE(DXL_HIWORD(goal_position))
                ]

                # Add Dynamixel goal position value to the Syncwrite parameter storage
                dxl_addparam_result = self.groupwrite_num.addParam(dxl_id, param_goal_position)
                if not dxl_addparam_result:
                    print(f"[ID:{dxl_id:03d}] groupSyncWrite addparam failed")
                    self.groupwrite_num.clearParam()
                    return False

            # Syncwrite goal position
            dxl_comm_result = self.groupwrite_num.txPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # Clear syncwrite parameter storage
            self.groupwrite_num.clearParam()

            # Syncread present position
            dxl_comm_result = self.groupread_num.txRxPacket()
            if dxl_comm_result != COMM_SUCCESS:
                print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

            # Check if groupsyncread data of Dynamixel#1 is available
            dxl_getdata_result = self.groupread_num.isAvailable(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL1_ID)
                quit()

            # Check if groupsyncread data of Dynamixel#2 is available
            dxl_getdata_result = self.groupread_num.isAvailable(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if dxl_getdata_result != True:
                print("[ID:%03d] groupSyncRead getdata failed" % DXL2_ID)
                quit()

            # # Check if groupsyncread data of Dynamixel#3 is available
            # dxl_getdata_result = self.groupread_num.isAvailable(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            # if dxl_getdata_result != True:
            #     print("[ID:%03d] groupSyncRead getdata failed" % DXL3_ID)
            #     quit()

            # Get Dynamixel#1 present position value
            dxl1_present_position = self.groupread_num.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            # Get Dynamixel#2 present position value
            dxl2_present_position = self.groupread_num.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            # # Get Dynamixel#3 present position value
            # dxl2_present_position = self.groupread_num.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            # print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n" % 
            #     (DXL1_ID, dxl_goal_position[DXL1_ID-1], dxl1_present_position, DXL2_ID, dxl_goal_position[DXL2_ID-1], dxl2_present_position, DXL3_ID, dxl_goal_position[DXL3_ID-1], dxl3_present_position))

            if not ((abs(dxl_goal_position[DXL1_ID-1] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) or (abs(dxl_goal_position[DXL2_ID-1] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD)):
                break
        # (abs(dxl_goal_position[DXL3_ID-1] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD)
            # Check for timeout - break loop if timeout is reached
            if time.time() - start_time > timeout:
                print("Timeout reached. Exiting loop.")
                break

    # print("[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\t[ID:%03d] GoalPos:%03d  PresPos:%03d\n" % 
    #             (DXL1_ID, dxl_goal_position[DXL1_ID-1], dxl1_present_position, DXL2_ID, dxl_goal_position[DXL2_ID-1], dxl2_present_position, DXL3_ID, dxl_goal_position[DXL3_ID-1], dxl3_present_position))

    def shutdown(self):
        self.disable_torque(DXL1_ID)
        self.disable_torque(DXL2_ID)
        # self.disable_torque(DXL3_ID)

        # Close port
        self.portHandler.closePort()

    def enable_torque(self, motor_id):
        # Enable Dynamixel Torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
            return False
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))
            return False
        else:
            print("Dynamixel#%d has been successfully connected" % motor_id)


        # Add parameter storage for Dynamixel present position value
        dxl_addparam_result = self.groupread_num.addParam(motor_id)
        if dxl_addparam_result != True:
            print("[ID:%03d] groupSyncRead addparam failed" % motor_id)
            quit()
        
        return True
    
    def disable_torque(self, motor_id):
        # Disable Dynamixel torque
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

# Convert angle to Dynamixel position (0-4095 range)
def angle_to_position(angle):
    return int((angle / 360) * 4095)
