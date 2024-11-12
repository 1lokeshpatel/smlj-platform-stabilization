from dynamixel_sdk import *  # Import the Dynamixel SDK library

# Control table address for XM series
ADDR_TORQUE_ENABLE = 64
ADDR_ID = 7

# Protocol version (check your motor manual, most use protocol 2.0)
PROTOCOL_VERSION = 2.0

# Default settings
BAUDRATE = 57600
DEVICENAME = '/dev/ttyUSB0'  # Change this to your port
CURRENT_ID = 1               # Current ID of the motor
NEW_ID = 2                   # New ID you want to set

TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
COMM_SUCCESS = 0

# Initialize PortHandler and PacketHandler
portHandler = PortHandler(DEVICENAME)
packetHandler = PacketHandler(PROTOCOL_VERSION)

# Open port
if not portHandler.openPort():
    print("Failed to open the port")
    quit()
if not portHandler.setBaudRate(BAUDRATE):
    print("Failed to change the baudrate")
    quit()

# Disable torque to allow ID change
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, CURRENT_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Error disabling torque: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Error disabling torque: {packetHandler.getRxPacketError(dxl_error)}")
else:
    print("Torque disabled successfully")

# Change the ID
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, CURRENT_ID, ADDR_ID, NEW_ID)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Error changing ID: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Error changing ID: {packetHandler.getRxPacketError(dxl_error)}")
else:
    print(f"Successfully changed ID from {CURRENT_ID} to {NEW_ID}")

# Enable torque again
dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, NEW_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
if dxl_comm_result != COMM_SUCCESS:
    print(f"Error enabling torque: {packetHandler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Error enabling torque: {packetHandler.getRxPacketError(dxl_error)}")
else:
    print("Torque enabled successfully")

# Close port
portHandler.closePort()
