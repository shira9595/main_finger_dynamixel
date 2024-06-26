import dynamixel_sdk as dxl
import random
import matplotlib.pyplot as plt
import numpy as np
import time

# Define your serial port
port = "COM5"  # Replace with your COM port

# Define motor settings
baudrate = 57600  # Adjust to your Dynamixel baudrate
motor_id = 1  # Replace with your motor ID
protocol_version = 2.0  # Protocol version of the Dynamixel model

# Initialize PacketHandler and PortHandler instances
port_handler = dxl.PortHandler(port)
packet_handler = dxl.PacketHandler(protocol_version)

def degrees_to_encoder_units(degrees):
    return int((degrees / 360.0) * 4095)

def encoder_units_to_degrees(encoder_units):
    return (encoder_units / 4095.0) * 360

def set_goal_position(port_handler, packet_handler, motor_id, goal_position_degrees):
    goal_position = degrees_to_encoder_units(goal_position_degrees)
    try:
        # Open port
        if not port_handler.openPort():
            print(f"Failed to open port {port}.")
            return False

        # Set baudrate
        if not port_handler.setBaudRate(baudrate):
            print(f"Failed to set baudrate to {baudrate}.")
            return False

        # Enable torque (Address 64 for TORQUE_ENABLE, for DYNAMIXEL X series)
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
            port_handler, motor_id, 64, 1)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print(f"Failed to enable torque: {packet_handler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            print(f"Error: {packet_handler.getRxPacketError(dxl_error)}")
            return False
        else:
            print(f"Torque enabled.")

        # Set goal position (Address 116 for GOAL_POSITION, for DYNAMIXEL X series)
        dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(
            port_handler, motor_id, 116, goal_position)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print(f"Failed to set goal position: {packet_handler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            print(f"Error: {packet_handler.getRxPacketError(dxl_error)}")
            return False
        else:
            print(f"Goal position set to: {goal_position_degrees} degrees ({goal_position} units)")

        return True

    finally:
        # Close port when done or in case of error
        port_handler.closePort()

def read_present_position(port_handler, packet_handler, motor_id):
    try:
        # Open port
        if not port_handler.openPort():
            print(f"Failed to open port {port}.")
            return None

        # Set baudrate
        if not port_handler.setBaudRate(baudrate):
            print(f"Failed to set baudrate to {baudrate}.")
            return None

        # Read present position (Address 132 for PRESENT_POSITION, for DYNAMIXEL X series)
        dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(
            port_handler, motor_id, 132)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print(f"Failed to read present position: {packet_handler.getTxRxResult(dxl_comm_result)}")
            return None
        elif dxl_error != 0:
            print(f"Error: {packet_handler.getRxPacketError(dxl_error)}")
            return None
        else:
            present_position_degrees = encoder_units_to_degrees(dxl_present_position)
            print(f"Present position: {dxl_present_position} units ({present_position_degrees:.2f} degrees)")
            return present_position_degrees

    finally:
        # Close port when done or in case of error
        port_handler.closePort()

desired_angles = []
received_angles = []

# Generate 100 random angles between 0 and 360 degrees
num_angles = 100
random_angles = [random.uniform(0, 360) for _ in range(num_angles)]

for desired_angle in random_angles:
    try:
        desired_angles.append(desired_angle)

        # Set goal position
        if set_goal_position(port_handler, packet_handler, motor_id, desired_angle):
            # Wait for the motor to reach the goal position
            time.sleep(1)  # Adjust sleep time as necessary
            received_angle = read_present_position(port_handler, packet_handler, motor_id)
            if received_angle is not None:
                received_angles.append(received_angle)

    finally:
        # Close port in case of any unexpected error
        port_handler.closePort()

# Plotting the results
plt.scatter(desired_angles, received_angles, label='Data points')
plt.xlabel('Desired Angle (degrees)')
plt.ylabel('Received Angle (degrees)')
plt.title('Desired vs Received Angle')

# Linear fitting
coefficients = np.polyfit(desired_angles, received_angles, 1)
linear_fit = np.poly1d(coefficients)
plt.plot(desired_angles, linear_fit(desired_angles), color='red', label=f'Fit: y={coefficients[0]:.2f}x + {coefficients[1]:.2f}')
plt.legend()
plt.grid(True)
plt.show()
