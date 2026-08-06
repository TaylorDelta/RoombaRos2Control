from numpy import pi
import serial
import time
import struct

# Setup the serial port (adjust the port name accordingly)
SERIAL_PORT = 'COM3'#'/dev/ttyUSB0'  # Change to your serial port (e.g., COMx on Windows)
BAUD_RATE = 115200

# Create a serial connection to Roomba
ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)



def request_sensor_data():
    """Send the Sensor command to Roomba and read the data."""
    print("Requesting sensor data...")
    ser.reset_input_buffer()
    ser.write(b'\x8E')  # Opcode 142 (Sensors)
    ser.write(b'\x00')  # Packet Code 0 (All sensor data)

    # Read the response (maximum 26 bytes for packet code 0)
    response = ser.read(26)
    if len(response) != 26:
        print("Error: Received incomplete data.")
        return None
    
    # Print raw response for debugging
    print("Raw sensor data:", list(response))
    
    # Parse the sensor data
    bumps_and_wheeldrops = response[0]
    bump_right = bumps_and_wheeldrops & 1  # Bit 0: Bump Right
    bump_left = (bumps_and_wheeldrops >> 1) & 1  # Bit 1: Bump Left
    wheel_drop_right = (bumps_and_wheeldrops >> 2) & 1  # Bit 2: Wheel Drop Right
    wheel_drop_left = (bumps_and_wheeldrops >> 3) & 1  # Bit 3: Wheel Drop Left
    wheel_drop_caster = (bumps_and_wheeldrops >> 4) & 1  # Bit 4: Wheel Drop Caster

    wall = response[1]  # 1 byte, wall sensor
    cliff_left = response[2]  # 1 byte, left cliff sensor
    cliff_front_left = response[3]  # 1 byte, front left cliff sensor
    cliff_front_right = response[4]  # 1 byte, front right cliff sensor
    cliff_right = response[5]  # 1 byte, right cliff sensor
    virtual_wall = response[6]  # 1 byte, virtual wall sensor
    motor_overcurrents = response[7]
    side_brush = motor_overcurrents & 1  # Bit 0: Side Brush
    vaccum = (motor_overcurrents >> 1) & 1  # Bit 1: Vacuum
    main_brush = (motor_overcurrents >> 2) & 1  # Bit 2: Main Brush
    drive_right = (motor_overcurrents >> 3) & 1  # Bit 3: Drive Right
    drive_left = (motor_overcurrents >> 4) & 1  # Bit 4: Drive Left

    dirt_left = response[8]  # 1 byte, dirt detector left
    dirt_right = response[9]  # 1 byte, dirt detector right
    remote_control = response[10]  # 1 byte, remote control command
    buttons = response[11]  # 1 byte, button states

    # Unpack 2-byte signed distance (mm) and angle (mm)
    distance = -struct.unpack('>h', response[12:14])[0]*10  # 2 bytes, signed distance (mm)
    angle = (2*struct.unpack('>h', response[14:16])[0])/235  # 2 bytes, signed angle (mm)

    charging_state = response[16]  # 1 byte, charging state

    # Map charging state to a description
    charging_state_mapping = {
        0: "Not Charging",
        1: "Charging Recovery",
        2: "Charging",
        3: "Trickle Charging",
        4: "Waiting",
        5: "Charging Error"
    }
    charging_state_description = charging_state_mapping.get(charging_state, "Unknown")

    # Unpack 2-byte unsigned values (voltage, current, charge, capacity)
    voltage = struct.unpack('>H', response[17:19])[0]  # 2 bytes, unsigned voltage (mV)
    current = struct.unpack('>h', response[19:21])[0]  # 2 bytes, signed current (mA)
    temperature = response[21]  # 1 byte, temperature (°C)
    charge = struct.unpack('>H', response[22:24])[0]  # 2 bytes, unsigned charge (mAh)
    capacity = struct.unpack('>H', response[24:26])[0]  # 2 bytes, unsigned capacity (mAh)

    # Print parsed data
    print(f"Bumps and Wheeldrops - Bump Left:", bump_left, "Bump Right:", bump_right, "Wheel Drop Left:", wheel_drop_left, "Wheel Drop Right:", wheel_drop_right, "Wheel Drop Caster:", wheel_drop_caster)   
    print(f"Wall Sensor: {wall}")
    print(f"Cliff Left: {cliff_left}")
    print(f"Cliff Front Left: {cliff_front_left}")
    print(f"Cliff Front Right: {cliff_front_right}")
    print(f"Cliff Right: {cliff_right}")
    print(f"Virtual Wall: {virtual_wall}")
    print(f"Motor Overcurrents - Side Brush:", side_brush, "Vacuum:", vaccum, "Main Brush:", main_brush, "Drive Right:", drive_right, "Drive Left:", drive_left)
    print(f"Dirt Left: {dirt_left}")
    print(f"Dirt Right: {dirt_right}")
    print(f"Remote Control Command: {remote_control}")
    print(f"Buttons Pressed: {buttons}")
    print(f"Distance Traveled: {distance} mm")
    print(f"Angle Turned: {angle} mm")
    print(f"Charging State: {charging_state_description}")
    print(f"Battery Voltage: {voltage} mV")
    print(f"Battery Current: {current} mA")
    print(f"Battery Temperature: {temperature} °C")
    print(f"Battery Charge: {charge} mAh")
    print(f"Battery Capacity: {capacity} mAh")

"""Request sensor data packet 101 (28 bytes)."""
def request_sensor_data_packet_101(): 
    print("Requesting sensor data packet 101...")
    cmd = bytearray()
    cmd += b'\x8E'  # 142
    cmd += b'\x65'  # 101
    ser.reset_input_buffer()
    ser.write(cmd)

    # Read the response (maximum 28 bytes for packet code 101)
    response = ser.read(28)
    if len(response) != 28:
        print("Error: Received incomplete data.")
        return None
    
    # Print raw response for debugging
    print("Raw sensor data packet 101:", list(response))

    encoder_counts_left = struct.unpack('>H', response[0:2])[0]
    encoder_counts_right = struct.unpack('>H', response[2:4])[0]
    # calculate distance from encoder counts
    # Constant(TICK_PER_REV=508.8, WHEEL_DIAMETER=72, WHEEL_BASE=235,TICK_TO_DISTANCE=0.44456499814949904317867595046408)
    tick_to_distance = (72*pi)/508.8  # in mm
    distance_left = encoder_counts_left * tick_to_distance
    distance_right = encoder_counts_right * tick_to_distance  # in mm
    light_bumper = response[4]
    light_bump_left = struct.unpack('>H', response[5:7])[0]
    light_bump_front_left = struct.unpack('>H', response[7:9])[0]
    light_bump_center_left = struct.unpack('>H', response[9:11])[0]
    light_bump_center_right = struct.unpack('>H', response[11:13])[0]
    light_bump_front_right = struct.unpack('>H', response[13:15])[0]
    light_bump_right = struct.unpack('>H', response[15:17])[0]
    ir_opcode_left = response[17]       
    ir_opcode_right = response[18]
    left_motor_current = struct.unpack('>h', response[19:21])[0]    
    right_motor_current = struct.unpack('>h', response[21:23])[0]
    main_brush_current = struct.unpack('>h', response[23:25])[0]
    side_brush_current = struct.unpack('>h', response[25:27])[0]
    stasis = response[27]

    # Print parsed data
    print(f"Encoder Counts Left: {encoder_counts_left}")
    print(f"Encoder Counts Right: {encoder_counts_right}")
    print(f"Distance Left: {distance_left} mm")
    print(f"Distance Right: {distance_right} mm")
    print(f"Light Bumper: {light_bumper}")
    print(f"Light Bump Left: {light_bump_left}")        
    print(f"Light Bump Front Left: {light_bump_front_left}")
    print(f"Light Bump Center Left: {light_bump_center_left}")  
    print(f"Light Bump Center Right: {light_bump_center_right}")
    print(f"Light Bump Front Right: {light_bump_front_right}")
    print(f"Light Bump Right: {light_bump_right}")
    print(f"IR Opcode Left: {ir_opcode_left}")
    print(f"IR Opcode Right: {ir_opcode_right}")
    print(f"Left Motor Current: {left_motor_current} mA")
    print(f"Right Motor Current: {right_motor_current} mA")
    print(f"Main Brush Current: {main_brush_current} mA")
    print(f"Side Brush Current: {side_brush_current} mA")
    print(f"Stasis: {stasis}")


def get_distance():
    # Serial sequence: [149][Number of Packets][Packet ID 1][Packet ID 2]...[Packet ID N]
    # send 149 2 43 44
    cmd = bytearray()
    cmd += b'\x05'  # Length
    cmd += b'\x95'  # Opcode
    cmd += b'\x02'  # Number of packets
    cmd += b'\x2B'  # Left encoder
    cmd += b'\x2C'  # Right encoder

    ser.reset_input_buffer()
    ser.write(cmd)

    # Read the response (4 bytes: 2 bytes for each encoder count)
    response = ser.read(4)
    if len(response) != 4:
        raise IOError("Incomplete encoder response")
    
    encoder_counts_left = struct.unpack('>H', response[0:2])[0]
    encoder_counts_right = struct.unpack('>H', response[2:4])[0]
    # calculate distance from encoder counts
    tick_to_distance = (72*pi)/508.8  # in mm
    distance_left = encoder_counts_left * tick_to_distance
    distance_right = encoder_counts_right * tick_to_distance  # in mm
    
    overall_distance = (distance_left + distance_right) / 2

    angle = (distance_right - distance_left) / 235 * (180/pi)  # in degrees

    print("=== Distance Data ===")
    print("Encoder Counts Left:", encoder_counts_left)
    print("Encoder Counts Right:", encoder_counts_right)
    print(f"Distance Left: {distance_left} mm")
    print(f"Distance Right: {distance_right} mm")
    print(f"Overall Distance: {overall_distance} mm")
    print(f"Angle Turned: {angle} degrees")
    print("=====================")
    
# Main function
def main():
    print("Connecting to Roomba...")
    time.sleep(2)  # Wait for connection to stabilize

    ser.write(b'\x80')  # Opcode 128 (Start)
    #time.sleep(0.1)

    # Request sensor data after SCI is started
    request_sensor_data()

    request_sensor_data_packet_101()

    ser.write(b'\x84')  # Opcode 132 Full Mode


    get_distance()

    
    #Serial sequence: [145] [Right velocity high byte] [Right velocity low byte] [Left velocity high byte][Left velocity low byte]
    # send left velocity = 200 and right velocity = 100
    # send command length 5, serial.write in one go
    cmd = bytearray()
    cmd += b'\x05'  # Length
    cmd += b'\x91'  # 145
    cmd += b'\x00'  # Right velocity high byte
    cmd += b'\x64'  # Right velocity low byte (100 in decimal)
    cmd += b'\x00'  # Left velocity high byte
    cmd += b'\xC8'  # Left velocity low byte (200 in decimal)

    ser.reset_input_buffer()
    ser.write(cmd)
    print("Sent drive command: Right velocity = 100 mm/s, Left velocity = 200 mm/s")

    # send 143
    #ser.write(b'\x8F')  # Opcode 143 (Stop)


    # # send commands [132],[137], [255], [56], [hex8000]
    # ser.write(b'\x84')  # Opcode 132 (Full Mode)
    # ser.write(b'\x89')  # Opcode 137 (Drive)
    # ser.write(b'\xFF')  # Velocity high byte
    # ser.write(b'\x38')  # Velocity low byte
    # ser.write(b'\x80')  # Radius high byte
    # ser.write(b'\x00')  # Radius low byte

    # wait 1 second
    time.sleep(5)

    

    # send commands [137], [0], [0], [hex8000]
    # ser.write(b'\x89')  # Opcode 137 (Drive)
    # ser.write(b'\x00')  # Velocity high byte
    # ser.write(b'\x00')  # Velocity low byte
    # ser.write(b'\x80')  # Radius high byte
    # ser.write(b'\x00')  # Radius low byte


    cmd = bytearray()
    cmd += b'\x05'        # Length
    cmd += b'\x91'        # Command ID
    cmd += b'\xFF'        # Right velocity high byte (-100)
    cmd += b'\x9C'        # Right velocity low byte
    cmd += b'\xFF'        # Left velocity high byte (-200)
    cmd += b'\x38'        # Left velocity low byte

    ser.reset_input_buffer()
    ser.write(cmd)
    print("Sent drive command: Right velocity = -100 mm/s, Left velocity = -200 mm/s")
    
    get_distance()


    # wait 1 second
    time.sleep(5)

    

    # send commands [137], [0], [0], [hex8000]
    # ser.write(b'\x89')  # Opcode 137 (Drive)
    # ser.write(b'\x00')  # Velocity high byte
    # ser.write(b'\x00')  # Velocity low byte
    # ser.write(b'\x80')  # Radius high byte
    # ser.write(b'\x00')  # Radius low byte


    cmd = bytearray()
    cmd += b'\x05'  # Length
    cmd += b'\x91'  # 145
    cmd += b'\x00'  # Right velocity high byte
    cmd += b'\x00'  # Right velocity low byte (0)
    cmd += b'\x00'  # Left velocity high byte
    cmd += b'\x00'  # Left velocity low byte (0)

    ser.reset_input_buffer()
    ser.write(cmd)
    print("Sent drive command: Right velocity = 100 mm/s, Left velocity = 200 mm/s")
    
    get_distance()

    request_sensor_data()

    request_sensor_data_packet_101()

    print("Stopping Roomba...")
    # send 173
    ser.write(b'\xAD')  # Opcode 173 (Stop)
    # Close the serial connection
    ser.close()


# Execute the main function
if __name__ == '__main__':
    main()
