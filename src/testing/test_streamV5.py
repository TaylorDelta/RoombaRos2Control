from numpy import pi
import serial
import time
import struct

# Setup the serial port (adjust the port name accordingly)
SERIAL_PORT = 'COM3'#'/dev/ttyUSB0'  # Change to your serial port (e.g., COMx on Windows)
BAUD_RATE = 115200

ser = serial.Serial(SERIAL_PORT, BAUD_RATE, timeout=1)
   
# Main function
def main():

    # Create a serial connection to Roomba

    time.sleep(2)  # Wait for connection to stabilize

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
        response = ser.read(ser.in_waiting)  # read everything in buffer
        # Take only the newest bytes
        t +=1

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
                        #f"Wall:{wall} VW:{virtual_wall} | "
                        #f"Motors SB:{side_brush} Vac:{vaccum} MB:{main_brush} DR:{drive_right} DL:{drive_left} | "
                        #f"Dirt L:{dirt_left} R:{dirt_right} | \n"
                        #f"Buttons:{buttons} RC:{remote_control} | "
                        f"Distance:{distance}mm Angle:{angle} | "
                        f"Charging:{charging_state_description} | "
                        f"V:{voltage}mV I:{current}mA T:{temperature}C \n"
                        f"Ch:{charge}mAh Cap:{capacity}mAh | "
                        #f"WallSig:{wall_signal} | "
                        #f"CliffSig L:{cliff_left_signal} FL:{cliff_front_left_signal} "
                        #f"FR:{cliff_front_right_signal} R:{cliff_right_signal} | \n"
                        #f"ChgAvail:{charger_available} OI:{oi_mode} | "
                        #f"Song:{song_number} Playing:{song_playing} | "
                        #f"StreamPkts:{oi_stream_num_packets} | \n"
                        f"Vel:{velocity}mm/s Rad:{radius}mm | "
                        f"VelR:{velocity_right} VelL:{velocity_left} | "
                        f"Enc L:{encoder_counts_left} R:{encoder_counts_right} | "
                        f"LightBump:{light_bumper} |\n"
                        f"LB L:{light_bump_left} FL:{light_bump_front_left} "
                        f"CL:{light_bump_center_left} CR:{light_bump_center_right} "
                        f"FR:{light_bump_front_right} R:{light_bump_right} | \n"
                        #f"IR L:{ir_opcode_left} R:{ir_opcode_right} | "
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
            ser.write(b'\x84')

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


        if t == 600:
            ser.write(b'\x84')

            cmd = bytearray()
            cmd += b'\x05'      # Length
            cmd += b'\x91'      # Command ID (145)
            cmd += b'\xFF'      # Right velocity high byte (-100)
            cmd += b'\x9C'      # Right velocity low byte
            cmd += b'\xFF'      # Left velocity high byte (-200)
            cmd += b'\x38'      # Left velocity low byte

            ser.reset_input_buffer()
            ser.write(cmd)
            print("Sent drive command: Right velocity = -100 mm/s, Left velocity = -200 mm/s")

        if t == 1000:
            ser.write(b'\x84')

            cmd = bytearray()
            cmd += b'\x05'  # Length
            cmd += b'\x91'  # 145
            cmd += b'\x00'  # Right velocity high byte
            cmd += b'\x00'  # Right velocity low byte (0)
            cmd += b'\x00'  # Left velocity high byte
            cmd += b'\x00'  # Left velocity low byte (0)

            ser.reset_input_buffer()
            ser.write(cmd)
            print("Sent drive command: Right velocity = 0 mm/s, Left velocity = 0 mm/s")

        if t > 1100:
            break


    cmd = bytearray([150,0])
    ser.write(cmd)


    time.sleep(2)    


    print("Stopping Roomba...")
    # send 173
    ser.write(b'\xAD')  # Opcode 173 (Stop)
    # Close the serial connection
    ser.close()


# Execute the main function
if __name__ == '__main__':
    main()
