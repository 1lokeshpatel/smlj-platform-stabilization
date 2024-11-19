# Motor Control Application Code

# TODO: Test with our motors 
# TODO: Figure out needed baudrate

import time
from dynamixel_sdk import *

# Control table address - specifically for XM430-W210
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52
ADDR_TORQUE_ENABLE      = 64
ADDR_GOAL_POSITION      = 116
ADDR_PRESENT_POSITION   = 132
ADDR_OPERATING_MODE     = 11
ADDR_PROFILE_VEL        = 112

# Data Byte Length
LEN_GOAL_POSITION       = 4
LEN_PRESENT_POSITION    = 4
LEN_PROFILE_VELOCITY    = 4

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

NUM_MOTORS                  = 3                             # Number of motors 
EXTENDED_POSITION_CONTROL_MODE = 4

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

        # Enable torque based on the number of motors
        self.enable_torque(DXL1_ID)
        self.enable_torque(DXL2_ID)
        if NUM_MOTORS == 3:
            self.enable_torque(DXL3_ID)

        return True

    # homing procedure - read current pos values, set initial pos, set velocity profile, move robot to start pos
    def calibrate(self):

        # Syncread present position
        dxl_comm_result = self.groupread_num.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixels are available
        for dxl_id in [DXL1_ID, DXL2_ID] if NUM_MOTORS == 2 else [DXL1_ID, DXL2_ID, DXL3_ID]:
            if not self.groupread_num.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                print(f"[ID:{dxl_id:03d}] groupSyncRead getdata failed")
                quit()

        # Get present position values
        dxl1_present_position = self.groupread_num.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        dxl2_present_position = self.groupread_num.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if NUM_MOTORS == 3:
            dxl3_present_position = self.groupread_num.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        # currently motors will be in a state that is in contact with base,
        # adding small increment to ensure initial position is slightly above contact point
        self.dxl1_initial_position = dxl1_present_position+5
        self.dxl2_initial_position = dxl2_present_position+5
        self.dxl3_initial_position = dxl3_present_position+5

        self.set_velocity_profile(DXL1_ID, 3)
        self.set_velocity_profile(DXL2_ID, 3)    
        self.set_velocity_profile(DXL3_ID, 3)    

        self.move_motor(angle_to_position(25), 
                        angle_to_position(25), 
                        angle_to_position(25))

        # # Syncwrite control mode
        # dxl_comm_result = self.groupwrite_num.txPacket()
        # if dxl_comm_result != COMM_SUCCESS:
        #     print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # # Clear syncwrite parameter storage
        # self.groupwrite_num.clearParam()

    def move_motor(self, motorPos1, motorPos2, motorPos3=0):
        # Present positions
        dxl1_present_position = 0                                   
        dxl2_present_position = 0
        dxl3_present_position = 0

        # [motorPos1, motorPos2, motorPos3]
        dxl_goal_position = [self.dxl1_initial_position+motorPos1, self.dxl2_initial_position+motorPos2]
        if NUM_MOTORS == 3:
            dxl_goal_position.append(self.dxl3_initial_position+motorPos3)

        timeout = 5  # Timeout in seconds
        start_time = time.time()

        while True:
            # Add goal position values to the SyncWrite parameter storage for each motor
            for i, dxl_id in enumerate([DXL1_ID, DXL2_ID] if NUM_MOTORS == 2 else [DXL1_ID, DXL2_ID, DXL3_ID]):
                goal_position = dxl_goal_position[i]
                param_goal_position = [
                    DXL_LOBYTE(DXL_LOWORD(goal_position)), 
                    DXL_HIBYTE(DXL_LOWORD(goal_position)), 
                    DXL_LOBYTE(DXL_HIWORD(goal_position)), 
                    DXL_HIBYTE(DXL_HIWORD(goal_position))
                ]

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

            # Check if groupsyncread data of Dynamixels are available
            for dxl_id in [DXL1_ID, DXL2_ID] if NUM_MOTORS == 2 else [DXL1_ID, DXL2_ID, DXL3_ID]:
                if not self.groupread_num.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                    print(f"[ID:{dxl_id:03d}] groupSyncRead getdata failed")
                    quit()

            # Get present position values
            dxl1_present_position = self.groupread_num.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            dxl2_present_position = self.groupread_num.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
            if NUM_MOTORS == 3:
                dxl3_present_position = self.groupread_num.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

            # Print present and goal positions
            if NUM_MOTORS == 2:
                print(
                    f"[ID:{DXL1_ID:03d}] GoalPos:{((dxl_goal_position[DXL1_ID-1]*360)/4095):04d}  PresPos:{((dxl1_present_position*360)/4095):04d}\t"
                    f"[ID:{DXL2_ID:03d}] GoalPos:{((dxl_goal_position[DXL2_ID-1]*360)/4095):04d}  PresPos:{((dxl2_present_position*360)/4095):04d}"
                )
            elif NUM_MOTORS == 3:
                print(
                    f"[ID:{DXL1_ID:03d}] GoalPos:{((dxl_goal_position[DXL1_ID-1]*360)/4095):04d}  PresPos:{((dxl1_present_position*360)/4095):04d}\t"
                    f"[ID:{DXL2_ID:03d}] GoalPos:{((dxl_goal_position[DXL2_ID-1]*360)/4095):04d}  PresPos:{((dxl2_present_position*360)/4095):04d}\t"
                    f"[ID:{DXL3_ID:03d}] GoalPos:{((dxl_goal_position[DXL3_ID-1]*360)/4095):04d}  PresPos:{((dxl3_present_position*360)/4095):04d}"
                )

            # Check if motors have reached the goal position
            if not (
                (abs(dxl_goal_position[DXL1_ID-1] - dxl1_present_position) > DXL_MOVING_STATUS_THRESHOLD) or
                (abs(dxl_goal_position[DXL2_ID-1] - dxl2_present_position) > DXL_MOVING_STATUS_THRESHOLD) or
                (NUM_MOTORS == 3 and abs(dxl_goal_position[DXL3_ID-1] - dxl3_present_position) > DXL_MOVING_STATUS_THRESHOLD)
            ):
                break

            # Timeout check
            if time.time() - start_time > timeout:
                print("Timeout reached. Exiting loop.")
                break

    def shutdown(self):
        self.disable_torque(DXL1_ID)
        self.disable_torque(DXL2_ID)
        if NUM_MOTORS == 3:
            self.disable_torque(DXL3_ID)

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

    def set_control_mode(self, motor_id, control_mode):
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(self.portHandler, motor_id, ADDR_OPERATING_MODE, control_mode)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def set_velocity_profile(self, motor_id, velocity):
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(self.portHandler, motor_id, ADDR_PROFILE_VEL, velocity)
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))
        elif dxl_error != 0:
            print("%s" % self.packetHandler.getRxPacketError(dxl_error))

    def read_status(self):
        # Syncread present position
        dxl_comm_result = self.groupread_num.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Check if groupsyncread data of Dynamixels are available
        for dxl_id in [DXL1_ID, DXL2_ID] if NUM_MOTORS == 2 else [DXL1_ID, DXL2_ID, DXL3_ID]:
            if not self.groupread_num.isAvailable(dxl_id, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION):
                print(f"[ID:{dxl_id:03d}] groupSyncRead getdata failed")
                quit()

        # Get present position values
        dxl1_present_position = self.groupread_num.getData(DXL1_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        dxl2_present_position = self.groupread_num.getData(DXL2_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)
        if NUM_MOTORS == 3:
            dxl3_present_position = self.groupread_num.getData(DXL3_ID, ADDR_PRESENT_POSITION, LEN_PRESENT_POSITION)

        # Print present and goal positions
        if NUM_MOTORS == 2:
            print(
                f"[ID:{DXL1_ID:03d}] PresPos:{dxl1_present_position:04d}\t"
                f"[ID:{DXL2_ID:03d}] PresPos:{dxl2_present_position:04d}"
            )
        elif NUM_MOTORS == 3:
            print(
                f"[ID:{DXL1_ID:03d}] PresPos:{(dxl1_present_position*360)/4095:04f}\t"
                f"[ID:{DXL2_ID:03d}] PresPos:{(dxl2_present_position*360)/4095:04f}\t"
                f"[ID:{DXL3_ID:03d}] PresPos:{(dxl3_present_position*360)/4095:04f}"
            )

    def read_velocity(self):
        # Syncread present position
        dxl_comm_result = self.groupread_num.txRxPacket()
        if dxl_comm_result != COMM_SUCCESS:
            print("%s" % self.packetHandler.getTxRxResult(dxl_comm_result))

        # Get velocity values
        dxl1_vel = self.groupread_num.getData(DXL1_ID, ADDR_PROFILE_VEL, LEN_PROFILE_VELOCITY)
        dxl2_vel = self.groupread_num.getData(DXL2_ID, ADDR_PROFILE_VEL, LEN_PROFILE_VELOCITY)
        if NUM_MOTORS == 3:
            dxl3_vel = self.groupread_num.getData(DXL3_ID, ADDR_PROFILE_VEL, LEN_PROFILE_VELOCITY)
        
        # Print velocity profile
        if NUM_MOTORS == 2:
            print(
                f"[ID:{DXL1_ID:03d}] Velocity:{dxl1_vel:04d}\t"
                f"[ID:{DXL2_ID:03d}] Velocity:{dxl2_vel:04d}"
            )
        elif NUM_MOTORS == 3:
            print(
                f"[ID:{DXL1_ID:03d}] Velocity:{(dxl1_vel*360)/4095:04f}\t"
                f"[ID:{DXL2_ID:03d}] Velocity:{(dxl2_vel*360)/4095:04f}\t"
                f"[ID:{DXL3_ID:03d}] Velocity:{(dxl3_vel*360)/4095:04f}"
            )

# Convert angle to Dynamixel position (0-4095 range)
def angle_to_position(angle):
    return int((angle / 360) * 4095)
