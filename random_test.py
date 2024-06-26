import dynamixel_sdk as dxl
import random
import matplotlib.pyplot as plt
import numpy as np
import time

class DynamixelMotor:
    def __init__(self, port, baudrate, motor_id, protocol_version):
        self.port = port
        self.baudrate = baudrate
        self.motor_id = motor_id
        self.protocol_version = protocol_version
        self.port_handler = dxl.PortHandler(port)
        self.packet_handler = dxl.PacketHandler(protocol_version)

    def degrees_to_encoder_units(self, degrees):
        return int((degrees / 360.0) * 4095)

    def encoder_units_to_degrees(self, encoder_units):
        return (encoder_units / 4095.0) * 360

    def open_port(self):
        if not self.port_handler.openPort():
            print(f"Failed to open port {self.port}.")
            return False
        if not self.port_handler.setBaudRate(self.baudrate):
            print(f"Failed to set baudrate to {self.baudrate}.")
            return False
        return True

    def close_port(self):
        self.port_handler.closePort()

    def enable_torque(self):
        dxl_comm_result, dxl_error = self.packet_handler.write1ByteTxRx(
            self.port_handler, self.motor_id, 64, 1)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print(f"Failed to enable torque: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            return False
        elif dxl_error != 0:
            print(f"Error: {self.packet_handler.getRxPacketError(dxl_error)}")
            return False
        print("Torque enabled.")
        return True

    def set_goal_position(self, goal_position_degrees):
        goal_position = self.degrees_to_encoder_units(goal_position_degrees)
        if not self.open_port():
            return False
        if not self.enable_torque():
            self.close_port()
            return False
        dxl_comm_result, dxl_error = self.packet_handler.write4ByteTxRx(
            self.port_handler, self.motor_id, 116, goal_position)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print(f"Failed to set goal position: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            self.close_port()
            return False
        elif dxl_error != 0:
            print(f"Error: {self.packet_handler.getRxPacketError(dxl_error)}")
            self.close_port()
            return False
        print(f"Goal position set to: {goal_position_degrees} degrees ({goal_position} units)")
        return True

    def read_present_position(self):
        if not self.open_port():
            return None
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, 132)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print(f"Failed to read present position: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            self.close_port()
            return None
        elif dxl_error != 0:
            print(f"Error: {self.packet_handler.getRxPacketError(dxl_error)}")
            self.close_port()
            return None
        present_position_degrees = self.encoder_units_to_degrees(dxl_present_position)
        print(f"Present position: {dxl_present_position} units ({present_position_degrees:.2f} degrees)")
        self.close_port()
        return present_position_degrees

def main():
    port = "COM5"
    baudrate = 57600
    motor_id = 1
    protocol_version = 2.0

    dynamixel = DynamixelMotor(port, baudrate, motor_id, protocol_version)

    desired_angles = []
    received_angles = []

    num_angles = 100
    random_angles = [random.uniform(0, 360) for _ in range(num_angles)]

    for desired_angle in random_angles:
        try:
            desired_angles.append(desired_angle)
            if dynamixel.set_goal_position(desired_angle):
                time.sleep(1)  # Adjust sleep time as necessary
                received_angle = dynamixel.read_present_position()
                if received_angle is not None:
                    received_angles.append(received_angle)
        finally:
            dynamixel.close_port()

    # plt.scatter(desired_angles, received_angles, label='Data points')
    # plt.xlabel('Desired Angle (degrees)')
    # plt.ylabel('Received Angle (degrees)')
    # plt.title('Desired vs Received Angle')
    #
    # coefficients = np.polyfit(desired_angles, received_angles, 1)
    # linear_fit = np.poly1d(coefficients)
    # plt.plot(desired_angles, linear_fit(desired_angles), color='red', label=f'Fit: y={coefficients[0]:.2f}x + {coefficients[1]:.2f}')
    # plt.legend()
    # plt.grid(True)
    # plt.show()

if __name__ == "__main__":
    main()