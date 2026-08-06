from numpy import pi
import serial
import time
import struct

# Setup the serial port (adjust the port name accordingly)
SERIAL_PORT = '/dev/ttyUSB0'#'COM3'  # Change to your serial port (e.g., COMx on Windows)
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

ser.write(b'\x80')  # Opcode 128 (Start)

ser.write(b'\x07')