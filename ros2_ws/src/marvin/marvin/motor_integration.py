import os
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import math

def dynamixel_motor(left_shoulder_flexion, left_shoulder_adduction, elbow):
    # Corrected Control Table Addresses for AX-18A and AX-12 \\300 degree max
    ADDR_AX_TORQUE_ENABLE = 35
    ADDR_AX_MOVING_SPEED = 32
    ADDR_AX_GOAL_POSITION = 30
    ADDR_MX_PRESENT_POSITION = 36
    SPEED = 100

    PROTOCOL_VERSION = 1.0

    AXL1_ID = 10                 # left_shoulder_flexion
    AXL2_ID = 0                 # left_shoulder_adduction
    AXL3_ID = 1                 # elbow

    angle_conversion = 180/math.pi

    # Baudrate & Device Name
    BAUDRATE = 1000000  # Ensure this matches your setup
    DEVICENAME = '/dev/ttyUSB0'  # May need adjustment based on your system

    # Initialization
    portHandler = PortHandler(DEVICENAME)
    packetHandler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if portHandler.openPort():
        print("Succeeded to open the port")
    else:
        print("Failed to open the port")
        return False

    # Set baudrate
    if portHandler.setBaudRate(BAUDRATE):
        print("Succeeded to change the baudrate")
    else:
        print("Failed to change the baudrate")
        return False

    #set moving speed
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AXL1_ID, ADDR_AX_MOVING_SPEED, SPEED)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AXL2_ID, ADDR_AX_MOVING_SPEED, SPEED)
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AXL3_ID, ADDR_AX_MOVING_SPEED, SPEED)
    if dxl_comm_result != COMM_SUCCESS:
        print("Failed to set moving speed")
    else:
        print(f"Successfully set moving speed to {SPEED}")

    # Enable torque for all motors
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, AXL1_ID, ADDR_AX_TORQUE_ENABLE, 1)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, AXL2_ID, ADDR_AX_TORQUE_ENABLE, 1)
    dxl_comm_result, dxl_error = packetHandler.write1ByteTxRx(portHandler, AXL3_ID, ADDR_AX_TORQUE_ENABLE, 1)

    # Convert radians to degrees and limit within 300 degrees
    left_shoulder_flexion = min(max(left_shoulder_flexion * angle_conversion, 0), 900)
    left_shoulder_adduction = min(max((left_shoulder_adduction + math.pi / 2) * angle_conversion, 0), 900)
    elbow = min(max((elbow + math.pi / 2) * angle_conversion, 0), 900)

    # Set goal positions for each motor
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AXL1_ID, ADDR_AX_GOAL_POSITION,
                                                              int(left_shoulder_flexion))
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AXL2_ID, ADDR_AX_GOAL_POSITION,
                                                              int(left_shoulder_adduction))
    dxl_comm_result, dxl_error = packetHandler.write2ByteTxRx(portHandler, AXL3_ID, ADDR_AX_GOAL_POSITION,
                                                              int(elbow))

    # Close port
    portHandler.closePort()

    return True

#dynamixel_motor(0,0,0)

#left_shoulder1= [20,100,300,500]
#left_shoulder2 = [80, 100, 300, 600]
#elbow = [100, 50, 500, 700]

#for i in left_shoulder1:
 #   for j in left_shoulder2:
  #      for k in elbow:
   #         dynamixel_motor(i, j, k)
            