import dynamixel_sdk as dxl
import serial
import time

class dynamixel:
    def __init__(self, port, baudrate, motor_id, protocol_version, port_handler, packet_handler):
        self.port="COM5"
        self.baudrate = 9600 #understand what about the 57600. if i need to change my defenition
        self.motor_id = 1
        self.protocol_version = 2.0
        self.port_handler = dxl.PortHandler(port) #is it correct here or should it be in Main?
        self.packet_handler = dxl.PacketHandler(protocol_version) #is it correct here or should it be in Main?

        ### should change what i have, and open the port and set the baude rate in the main. here i need to deal with try execpt if it doesnt open up##

    def connect(self):
        try:
            if self.port_handler.openPort():
                print ("Port has opened successfully")
            else:
                raise IOError (f"Port failed to open: {self.port}")

            if self.port_hanlder.setBaudRate(self.baudrate):
                print("Baudrate set successfully")
            else:
                raise   IOError (f" failed to set baudrate: {self.baudrate}")

            ##self.dxl=dxl.Dxl(self.port,self.baudrate)
        except:
            print("Error connecting to Dynamixel - opening port or setting baudrate")

    def set_goal_position(self,goal_position):
        try:
            dxl_com_result, dxl_error = self.packet_handler.write1Byte(self.port_handler,self.motor_id,64,1)
            print ("torque enabled")
            dxl_com_result, dxl_error = self.packet_handler.write4Byte(self.port_handler,self.motor_id,116,goal_position)
            print(f"Goal position set to: {goal_position}")
        finally:
            self.port_handler.closePort()

    def read_position(self):
        try:
            dxl_present_position, dxl_com_result, dxl_error = self.packet_handler.read4Byte(self.port_handler,self.motor_id,132)
            print(f"Present position: {dxl_present_position}")
            #i need to add a line of if the posotion havent changed?
            #why in the chat example i need to put a return
        finally:
            self.port_handler.closePort()

class Arduino_Communication:
    def __init__(self, port, baud_rate=9600):
        self.port = port
        self.baud_rate = baud_rate
        self.ser = None
        self.connect()

    def connect(self):
        try:
            self.ser = serial.Serial(self.port, self.baud_rate)
            time.sleep(2)  # Allow time for the connection to establish
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

class Vibration_Motor:
    def __init__(self, arduino):
        self.arduino = arduino

    def activate_vibrationmotor(self):
        self.arduino.write_line("activate_vibrationmotor\n")
        print("Activating vibration motor")

        