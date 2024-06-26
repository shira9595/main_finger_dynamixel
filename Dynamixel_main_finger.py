import serial
import time
import dynamixel_sdk as dxl

class DynamixelMotor:
    def __init__(self, port, baudrate, motor_id, protocol_version):
        self.port = port
        self.baudrate = baudrate
        self.motor_id = motor_id
        self.protocol_version = protocol_version
        self.port_handler = dxl.PortHandler(port) #initalizing the port
        self.packet_handler = dxl.PacketHandler(protocol_version) #initiatlizing the protocol version

    def degrees_to_encoder_units(self, degrees):
        return int((degrees / 360.0) * 4095)

    def encoder_units_to_degrees(self, encoder_units):
        return (encoder_units / 4095.0) * 360

    def connect(self):
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
        dxl_present_position, dxl_comm_result, dxl_error = self.packet_handler.read4ByteTxRx(
            self.port_handler, self.motor_id, 132)
        if dxl_comm_result != dxl.COMM_SUCCESS:
            print(f"Failed to read present position: {self.packet_handler.getTxRxResult(dxl_comm_result)}")
            return None
        elif dxl_error != 0:
            print(f"Error: {self.packet_handler.getRxPacketError(dxl_error)}")
            return None
        present_position_degrees = self.encoder_units_to_degrees(dxl_present_position)
        print(f"Present position: {dxl_present_position} units ({present_position_degrees:.2f} degrees)")
        return present_position_degrees

class ArduinoCommunication:
    def __init__(self, arduino_port, baud_rate=57600):
        self.arduino_port = arduino_port
        self.baud_rate = baud_rate
        self.ser = None
        self.connect()

    def connect(self):
        try:
            self.ser = serial.Serial(self.arduino_port, self.baud_rate)
            print("Arduino connected successfully")
        except serial.SerialException as e:
            print(f"Error opening serial port: {e}")
            self.ser = None

    def close_connection(self):
        if self.ser and self.ser.is_open:
            self.ser.close()

    def read_line(self):
        if self.ser and self.ser.is_open:
            return self.ser.readline().decode().strip()
        return ""

    def write_line(self, line):
        if self.ser and self.ser.is_open:
            self.ser.write(line.encode())
            print(f"Sent to Arduino: {line}")

class VibrationMotor:
    def __init__(self, arduino):
        self.arduino = arduino

    def activate_vibration_motor(self):
        self.arduino.write_line("activate_vibrationmotor\n")
        print("Activating vibration motor")
        time.sleep(1)  # Allow time for the command to be processed

def main():
    arduino_port = "COM4"
    arduino = ArduinoCommunication(arduino_port)
    vibration_motor = VibrationMotor(arduino)

    dxl_port = "COM5"
    baudrate = 57600 ##so where does the 9600 expressed?
    motor_id = 1
    protocol_version = 2.0

    dynamixel = DynamixelMotor(dxl_port, baudrate, motor_id, protocol_version)
    time.sleep(1)

    try:
        while True:
            desired_angle = float(input("Please enter the desired angle: "))
            if dynamixel.set_goal_position(desired_angle):
                time.sleep(1) ##think about what yoni said about this
                actual_angle = round(dynamixel.read_present_position(),2)
                if actual_angle is not None:
                    angle_error = round(abs(desired_angle - actual_angle),2)
                    print(f"Relative angle error: {angle_error}")
                    print(f"Current angle: {actual_angle}")

                    if angle_error > 0.0:  # Add tolerance for angle error
                        vibration_motor.activate_vibration_motor()
                        arduino_feedback = arduino.read_line()
                        print(f"Arduino feedback: {arduino_feedback}")
                        if arduino_feedback == "vibration motor activated 3 sec as wanted":
                            print("Vibration motor was activated for 3 sec as wanted")
                        else:
                            print("Vibration motor wasn't activated")
                    else:
                        print("Motor's angle hasn't changed")

    except KeyboardInterrupt:
        print("Manually exiting program")
    finally:
        arduino.close_connection()
        dynamixel.close_port()

if __name__ == "__main__":
    main()
