# Motor Control Application Code

# TODO: Test with our motors 
# TODO: Figure out needed baudrate
# TODO: Modularize
# TODO: How to convert angle to position value

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

# Initialize PortHandler Structs
# Set the port path
# Get methods and members of PortHandlerLinux or PortHandlerWindows
port_num = dynamixel.portHandler(DEVICENAME)

# Initialize PacketHandler Structs
dynamixel.packetHandler()

# Initialize Groupsyncwrite instance
groupwrite_num = dynamixel.groupSyncWrite(port_num, PROTOCOL_VERSION, ADDR_PRO_GOAL_POSITION, LEN_PRO_GOAL_POSITION)

# Initialize Groupsyncread Structs for Present Position
groupread_num = dynamixel.groupSyncRead(port_num, PROTOCOL_VERSION, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

index = 0
dxl_comm_result = COMM_TX_FAIL                              # Communication result
dxl_addparam_result = 0                                     # AddParam result
dxl_getdata_result = 0                                      # GetParam result

# [motor1Angle, motor2Angle, motor3Angle]
dxl_goal_position = [310, 310, 310]                         # Goal position - currently using test position of 310

dxl_error = 0                                               # Dynamixel error

# Present positions
dxl1_present_position = 0                                   
dxl2_present_position = 0
dxl3_present_position = 0

# Open port
if dynamixel.openPort(port_num):
    print("Succeeded to open the port!")

# Set port baudrate
if dynamixel.setBaudRate(port_num, BAUDRATE):
    print("Succeeded to change the baudrate!")

# Enable Dynamixel#1 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
else:
    print("Dynamixel#1 has been successfully connected")

# Enable Dynamixel#2 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
else:
    print("Dynamixel#2 has been successfully connected")
    
# Enable Dynamixel#3 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_ENABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))
else:
    print("Dynamixel#3 has been successfully connected")

# Add parameter storage for Dynamixel#1 present position value
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL1_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL1_ID))
    quit()

# Add parameter storage for Dynamixel#2 present position value
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL2_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL2_ID))
    quit()

# Add parameter storage for Dynamixel#3 present position value
dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncReadAddParam(groupread_num, DXL3_ID)).value
if dxl_addparam_result != 1:
    print("[ID:%03d] groupSyncRead addparam failed" % (DXL3_ID))
    quit()

timeout = 5  # Timeout in seconds
start_time = time.time()

while 1:

    # Add Dynamixel#1 goal position value to the Syncwrite storage
    dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_num, DXL1_ID, dxl_goal_position[index], LEN_PRO_GOAL_POSITION)).value
    print(dxl_addparam_result)
    if dxl_addparam_result != 1:
        print("[ID:%03d] groupSyncWrite addparam failed" % (DXL1_ID))
        quit()

    # Add Dynamixel#2 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_num, DXL2_ID, dxl_goal_position[index], LEN_PRO_GOAL_POSITION)).value
    if dxl_addparam_result != 1:
        print("[ID:%03d] groupSyncWrite addparam failed" % (DXL2_ID))
        quit()
    
    # Add Dynamixel#3 goal position value to the Syncwrite parameter storage
    dxl_addparam_result = ctypes.c_ubyte(dynamixel.groupSyncWriteAddParam(groupwrite_num, DXL3_ID, dxl_goal_position[index], LEN_PRO_GOAL_POSITION)).value
    if dxl_addparam_result != 1:
        print("[ID:%03d] groupSyncWrite addparam failed" % (DXL3_ID))
        quit()

    # Syncwrite goal position
    dynamixel.groupSyncWriteTxPacket(groupwrite_num)
    if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
        dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

    # Clear syncwrite parameter storage
    dynamixel.groupSyncWriteClearParam(groupwrite_num)

    while 1:
        # Syncread present position
        dynamixel.groupSyncReadTxRxPacket(groupread_num)
        if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
            dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))

        # Check if groupsyncread data of Dynamixel#1 is available
        dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)).value
        if dxl_getdata_result != 1:
            print("[ID:%03d] groupSyncRead getdata failed" % (DXL1_ID))
            quit()

        # Check if groupsyncread data of Dynamixel#2 is available
        dxl_getdata_result = ctypes.c_ubyte(dynamixel.groupSyncReadIsAvailable(groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)).value
        if dxl_getdata_result != 1:
            print("[ID:%03d] groupSyncRead getdata failed" % (DXL2_ID))
            quit()

        # Get Dynamixel#1 present position value
        dxl1_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL1_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#2 present position value
        dxl2_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL2_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

        # Get Dynamixel#3 present position value
        dxl3_present_position = dynamixel.groupSyncReadGetData(groupread_num, DXL3_ID, ADDR_PRO_PRESENT_POSITION, LEN_PRO_PRESENT_POSITION)

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

# Disable Dynamixel#1 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL1_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Disable Dynamixel#2 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL2_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Disable Dynamixel#3 Torque
dynamixel.write1ByteTxRx(port_num, PROTOCOL_VERSION, DXL3_ID, ADDR_PRO_TORQUE_ENABLE, TORQUE_DISABLE)
if dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION) != COMM_SUCCESS:
    dynamixel.printTxRxResult(PROTOCOL_VERSION, dynamixel.getLastTxRxResult(port_num, PROTOCOL_VERSION))
elif dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION) != 0:
    dynamixel.printRxPacketError(PROTOCOL_VERSION, dynamixel.getLastRxPacketError(port_num, PROTOCOL_VERSION))

# Close port
dynamixel.closePort(port_num)
