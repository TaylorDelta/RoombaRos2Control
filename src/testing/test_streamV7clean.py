from numpy import pi
import serial
import time
import struct

# Setup the serial port (adjust the port name accordingly)
SERIAL_PORT = '/dev/ttyUSB0'#'COM3'  # Change to your serial port (e.g., COMx on Windows)
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)

# ==== Playground ====

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

def reconnect_serial():
    print("Reconnecting serial port...")

    try:
        ser.write(b'\x80')  # Opcode 128 (Start)
        ser.write(b'\x84')
        print("send 128")
        ser.read()
    except Exception:
        pass  # ignore close errors

    try:
        if ser and ser.is_open:
            ser.close()
            print('ser.close()')
    except Exception:
        pass  # ignore close errors

    time.sleep(1)  # give OS time to release the port

    while True:
        try:
            ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
            print("Serial reconnected")
            ser.write(b'\x80')  # Opcode 128 (Start)
            ser.write(b'\x84')
            cmd = bytearray([148, 1, 100])
            ser.write(cmd)

        except serial.SerialException:
            print("Retrying connection...")
            time.sleep(2)

# Main function
def main():

    # Create a serial connection to Roomba
    ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)


    ser.write(b'\x80')  # Opcode 128 (Start)

    print('Start Roomba')

    ser.write(b'\x84')

    print('Set to Full Mode')

    # start data stream
    # Serial sequence: [148] [Number of packets] [Packet ID 1] [Packet ID 2] [Packet ID 3]
    #cmd = bytearray([148, 2, 43, 44])
    cmd = bytearray([148, 1, 100])

    ser.write(cmd)

    time.sleep(2)

    t = 1

    while True:
        time.sleep(0.03) 
        t +=1
        print(f"|==={t}===|")
        try:
            response = ser.read(ser.in_waiting)

        except OSError as e:
            print("Serial device error:", e)
            break#//ser = reconnect_serial()
        # Take only the newest bytes

        # Iterate over bytes to find start of packet (packet ID 19)
        for idx in range(len(response)-1, -1, -1):
            if response[idx] == 19 and idx <= len(response) - 84:
                packet_bytes = response[idx:idx+84]
                checksum = sum(packet_bytes) & 0xFF
                if checksum == 0:
                    packet = response[idx+3:idx+80+3]
                    # Parse the packet
                    bumps_and_wheeldrops = packet[0]
                    bump_right = bumps_and_wheeldrops & 1
                    bump_left = (bumps_and_wheeldrops >> 1) & 1
                    wheel_drop_right = (bumps_and_wheeldrops >> 2) & 1
                    wheel_drop_left = (bumps_and_wheeldrops >> 3) & 1
                    wheel_drop_caster = (bumps_and_wheeldrops >> 4) & 1

                    wall = packet[1]
                    cliff_left = packet[2]
                    cliff_front_left = packet[3]
                    cliff_front_right = packet[4]
                    cliff_right = packet[5]
                    virtual_wall = packet[6]

                    motor_overcurrents = packet[7]
                    side_brush = motor_overcurrents & 1
                    vaccum = (motor_overcurrents >> 1) & 1
                    main_brush = (motor_overcurrents >> 2) & 1
                    drive_right = (motor_overcurrents >> 3) & 1
                    drive_left = (motor_overcurrents >> 4) & 1

                    dirt_left = packet[8]
                    dirt_right = packet[9]
                    remote_control = packet[10]
                    buttons = packet[11]

                    distance = -struct.unpack('>h', packet[12:14])[0]*10
                    angle = (2*struct.unpack('>h', packet[14:16])[0])/235

                    charging_state = packet[16]
                    charging_state_mapping = {
                        0: "Not Charging",
                        1: "Charging Recovery",
                        2: "Charging",
                        3: "Trickle Charging",
                        4: "Waiting",
                        5: "Charging Error"
                    }
                    charging_state_description = charging_state_mapping.get(charging_state, "Unknown")

                    voltage = struct.unpack('>H', packet[17:19])[0]
                    current = struct.unpack('>h', packet[19:21])[0]
                    temperature = packet[21]
                    charge = struct.unpack('>H', packet[22:24])[0]
                    capacity = struct.unpack('>H', packet[24:26])[0]

                    wall_signal = struct.unpack('>H', packet[26:28])[0]
                    cliff_left_signal = struct.unpack('>H', packet[28:30])[0]
                    cliff_front_left_signal = struct.unpack('>H', packet[30:32])[0]
                    cliff_front_right_signal = struct.unpack('>H', packet[32:34])[0]
                    cliff_right_signal = struct.unpack('>H', packet[34:36])[0]

                    unused_32 = packet[36]
                    unused_33 = struct.unpack('>H', packet[37:39])[0]
                    charger_available = packet[39]
                    oi_mode = packet[40]
                    song_number = packet[41]
                    song_playing = packet[42]
                    oi_stream_num_packets = packet[43]
                    velocity = struct.unpack('>h', packet[44:46])[0]
                    radius = struct.unpack('>h', packet[46:48])[0]
                    velocity_right = struct.unpack('>h', packet[48:50])[0]
                    velocity_left = struct.unpack('>h', packet[50:52])[0]

                    encoder_counts_left = struct.unpack('>H', packet[52:54])[0]
                    encoder_counts_right = struct.unpack('>H', packet[54:56])[0]

                    light_bumper = packet[56]
                    light_bump_left = struct.unpack('>H', packet[57:59])[0]
                    light_bump_front_left = struct.unpack('>H', packet[59:61])[0]
                    light_bump_center_left = struct.unpack('>H', packet[61:63])[0]
                    light_bump_center_right = struct.unpack('>H', packet[63:65])[0]
                    light_bump_front_right = struct.unpack('>H', packet[65:67])[0]
                    light_bump_right = struct.unpack('>H', packet[67:69])[0]
                    ir_opcode_left = packet[69]
                    ir_opcode_right = packet[70]
                    #left_motor_current = struct.unpack('>h', packet[71:73])[0]
                    #right_motor_current = struct.unpack('>h', packet[73:75])[0]
                    #main_brush_current = struct.unpack('>h', packet[75:77])[0]
                    #side_brush_current = struct.unpack('>h', packet[77:79])[0]
                    #stasis = packet[79]


                    # Print parsed data live
                    print(
                        f"\rT: {t} | len {len(packet)} | len buffer {len(response)}"
                        f"Bumps L:{bump_left} R:{bump_right} | "
                        f"Wheel Drop L:{wheel_drop_left} R:{wheel_drop_right} C:{wheel_drop_caster} | "
                        f"Cliffs L:{cliff_left} FL:{cliff_front_left} FR:{cliff_front_right} R:{cliff_right} | \n"
                        f"Wall:{wall} VW:{virtual_wall} | "
                        f"Motors SB:{side_brush} Vac:{vaccum} MB:{main_brush} DR:{drive_right} DL:{drive_left} | "
                        f"Dirt L:{dirt_left} R:{dirt_right} | \n"
                        f"Buttons:{buttons} RC:{remote_control} | "
                        f"Distance:{distance}mm Angle:{angle} | "
                        f"Charging:{charging_state_description} | "
                        f"V:{voltage}mV I:{current}mA T:{temperature}C \n"
                        f"Ch:{charge}mAh Cap:{capacity}mAh | "
                        f"WallSig:{wall_signal} | "
                        f"CliffSig L:{cliff_left_signal} FL:{cliff_front_left_signal} "
                        f"FR:{cliff_front_right_signal} R:{cliff_right_signal} | \n"
                        f"ChgAvail:{charger_available} OI:{oi_mode} | "
                        f"Song:{song_number} Playing:{song_playing} | "
                        f"StreamPkts:{oi_stream_num_packets} | \n"
                        f"Vel:{velocity}mm/s Rad:{radius}mm | "
                        f"VelR:{velocity_right} VelL:{velocity_left} | "
                        f"Enc L:{encoder_counts_left} R:{encoder_counts_right} | "
                        f"LightBump:{light_bumper} |\n"
                        f"LB L:{light_bump_left} FL:{light_bump_front_left} "
                        f"CL:{light_bump_center_left} CR:{light_bump_center_right} "
                        f"FR:{light_bump_front_right} R:{light_bump_right} | \n"
                        f"IR L:{ir_opcode_left} R:{ir_opcode_right} | "
                        #f"Curr L:{left_motor_current}mA R:{right_motor_current}mA \n"
                        #f"MB:{main_brush_current}mA SB:{side_brush_current}mA | "
                        #f"Stasis:{stasis}",
                        ,
                        end="",
                        flush=True
                    )
                    break
            else:
                continue
        if t == 100:
            print('start cleaning')
            ser.write(b'\x87')
            # ser.write(b'\x84')

            # cmd = bytearray()
            # cmd += b'\x05'  # Length
            # cmd += b'\x91'  # 145
            # cmd += b'\x00'  # Right velocity high byte
            # cmd += b'\x64'  # Right velocity low byte (100 in decimal)
            # cmd += b'\x00'  # Left velocity high byte
            # cmd += b'\xC8'  # Left velocity low byte (200 in decimal)

            # ser.reset_input_buffer()
            # ser.write(cmd)
            # print("Sent drive command: Right velocity = 100 mm/s, Left velocity = 200 mm/s")

        if t == 1000:
            print('stop cleaning')
            ser.write(b'\x80')
            # ser.write(b'\x84')

            # cmd = bytearray()
            # cmd += b'\x05'  # Length
            # cmd += b'\x91'  # 145
            # cmd += b'\x00'  # Right velocity high byte
            # cmd += b'\x00'  # Right velocity low byte (0)
            # cmd += b'\x00'  # Left velocity high byte
            # cmd += b'\x00'  # Left velocity low byte (0)

            # ser.reset_input_buffer()
            # ser.write(cmd)
            # print("Sent drive command: Right velocity = 0 mm/s, Left velocity = 0 mm/s")

        if t > 1100:
            break


    cmd = bytearray([150,0])
    ser.write(cmd)
    print('Stopping stream')
    ser.reset_input_buffer()



    time.sleep(2)

    request_sensor_data()

    


    print("Stopping Roomba...")
    # send 173
    ser.write(b'\xAD')  # Opcode 173 (Stop)
    # Close the serial connection
    ser.write(b'\x07')

    time.sleep(5)


    ser.close()


# Execute the main function
if __name__ == '__main__':
    main()
