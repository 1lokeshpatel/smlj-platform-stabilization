import sys
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control table addresses for Position Limits
ADDR_MAX_POSITION_LIMIT = 48
ADDR_MIN_POSITION_LIMIT = 52
ADDR_TORQUE_ENABLE      = 64
LEN_POSITION_LIMIT      = 4

# Protocol version
PROTOCOL_VERSION        = 2

# Default settings
BAUDRATE                = 57600
DEVICENAME              = "/dev/ttyUSB0"  # TODO: Adjust this if needed
DXL_ID                  = 1               # Default motor ID to configure, can be passed as an argument

TORQUE_DISABLE          = 0               # Value for disabling torque
COMM_SUCCESS            = 0               # Communication Success result value

class PositionLimiter:
    def __init__(self, dxl_id, min_limit, max_limit):
        self.portHandler = PortHandler(DEVICENAME)
        self.packetHandler = PacketHandler(PROTOCOL_VERSION)
        self.dxl_id = dxl_id
        self.min_limit = min_limit
        self.max_limit = max_limit

    def setup(self):
        # Open port
        if not self.portHandler.openPort():
            print("Failed to open the port")
            return False
        print("Succeeded to open the port!")

        # Set port baudrate
        if not self.portHandler.setBaudRate(BAUDRATE):
            print("Failed to set baudrate")
            return False
        print("Succeeded to set baudrate!")

        # Disable torque before setting limits
        dxl_comm_result, dxl_error = self.packetHandler.write1ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to disable torque for Motor ID {self.dxl_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            print(f"Error disabling torque for Motor ID {self.dxl_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            return False

        return True

    def set_position_limits(self):
        # Set Maximum Position Limit
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_MAX_POSITION_LIMIT, self.max_limit
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to set max position limit for Motor ID {self.dxl_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            print(f"Error while setting max position limit for Motor ID {self.dxl_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            return False

        # Set Minimum Position Limit
        dxl_comm_result, dxl_error = self.packetHandler.write4ByteTxRx(
            self.portHandler, self.dxl_id, ADDR_MIN_POSITION_LIMIT, self.min_limit
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to set min position limit for Motor ID {self.dxl_id}: {self.packetHandler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            print(f"Error while setting min position limit for Motor ID {self.dxl_id}: {self.packetHandler.getRxPacketError(dxl_error)}")
            return False

        print(f"Position limits set for Motor ID {self.dxl_id}: Min={self.min_limit}, Max={self.max_limit}")
        return True

    def shutdown(self):
        # Close port
        self.portHandler.closePort()
        print("Port closed successfully")

if __name__ == "__main__":
    # Command-line arguments: Motor ID, min limit, max limit
    if len(sys.argv) != 4:
        print("Usage: python set_position_limits.py [MOTOR_ID] [MIN_LIMIT] [MAX_LIMIT]")
        sys.exit(1)

    motor_id = int(sys.argv[1])
    min_limit = int(sys.argv[2])
    max_limit = int(sys.argv[3])

    # Create PositionLimiter instance
    limiter = PositionLimiter(motor_id, min_limit, max_limit)

    if not limiter.setup():
        print("Setup failed. Exiting.")
        sys.exit(1)

    # Set the position limits
    if limiter.set_position_limits():
        print("Position limits configured successfully")

    # Shutdown and close port
    limiter.shutdown()
