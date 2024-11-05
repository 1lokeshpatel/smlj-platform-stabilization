/*
Motor Control Application: 
    - Moves motor value by value of TEST_GOAL_POS

TODO: 
    - Build and test: Install Dynamixel SDK, compile with CMake, and run with Raspberry Pi
    - Create shared variables for three motor angles and move motors when those variables are altered by CV program
    - Make more modular by converting into motor control class
*/
#include <stdlib.h>
#include <stdio.h>

#include "dynamixel_sdk.h"                                  // Uses DYNAMIXEL SDK library

// Control table addresses - specifically for XM430-W210
#define ADDR_TORQUE_ENABLE           64
#define ADDR_GOAL_POSITION           116
#define ADDR_PRESENT_POSITION        132

// Protocol version
#define PROTOCOL_VERSION            2.0

// Default setting
#define DXL_ID                       1
#define BAUDRATE                    57600
#define DEVICENAME                  "/dev/ttyUSB0"

#define TORQUE_ENABLE                   1                   // Value for enabling the torque
#define TORQUE_DISABLE                  0                   // Value for disabling the torque

#define TEST_GOAL_POS                   310                 // Angle / 0.29 == Dynamixel position value
#define DXL_MOVING_STATUS_THRESHOLD     20                  // Dynamixel moving status threshold

int main(int argc, char **argv){

    // Initialize PortHandler instance
    // Set the port path
    // Get methods and members of PortHandlerLinux or PortHandlerWindows
    dynamixel::PortHandler *portHandler = dynamixel::PortHandler::getPortHandler(DEVICENAME);

    // Initialize PacketHandler instance
    // Set the protocol version
    // Get methods and members of Protocol1PacketHandler or Protocol2PacketHandler
    dynamixel::PacketHandler *packetHandler = dynamixel::PacketHandler::getPacketHandler(PROTOCOL_VERSION);
  
    uint8_t dxl_error = 0;                          // Dynamixel error
    int32_t dxl_present_position = 0;               // Present position

    // Open port
    if (!portHandler->openPort())
    {
        std::cout << "Failed to open the port!" << endl;
        return 1;
    }

    // Set port baudrate
    if (portHandler->setBaudRate(BAUDRATE))
    {
        std::cout << "Succeeded to change the baudrate to " << BAUDRATE << "!"<< endl;
    }
    else
    {
        std::cout << "Failed to change the baudrate!" << endl;
        return 1;
    }

    // Enable Dynamixel Torque
    uint8_t dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_ENABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        cout << "Failed to enable torque for Dynamixel!" << endl;
        return 1;
    }

    int32_t dxl_goal_position = TEST_GOAL_POS;
    // Write goal position
    dxl_comm_result = packetHandler->write4ByteTxRx(portHandler, DXL_ID, ADDR_GOAL_POSITION, dxl_goal_position, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
      packetHandler->printTxRxResult(dxl_comm_result);
    }
    else if (dxl_error != 0)
    {
      packetHandler->printRxPacketError(dxl_error);
    }

    do
    {
      // Read present position
      dxl_comm_result = packetHandler->read4ByteTxRx(portHandler, DXL_ID, ADDR_PRESENT_POSITION, (uint32_t*)&dxl_present_position, &dxl_error);
      if (dxl_comm_result != COMM_SUCCESS)
      {
        packetHandler->printTxRxResult(dxl_comm_result);
      }
      else if (dxl_error != 0)
      {
        packetHandler->printRxPacketError(dxl_error);
      }

      printf("[ID:%03d] GoalPos:%03d  PresPos:%03d\n", DXL_ID, dxl_goal_position, dxl_present_position);

    } while((abs(dxl_goal_position - dxl_present_position) > DXL_MOVING_STATUS_THRESHOLD));

    printf("Motor has successfully moved to goal position!\n");

    // Disable Dynamixel Torque
    dxl_comm_result = packetHandler->write1ByteTxRx(portHandler, DXL_ID, ADDR_TORQUE_ENABLE, TORQUE_DISABLE, &dxl_error);
    if (dxl_comm_result != COMM_SUCCESS)
    {
        printf("Failed to disable torque for Dynamixel!\n");
        return 1;
    }

    // Close port
    portHandler->closePort();

    return 0;
}