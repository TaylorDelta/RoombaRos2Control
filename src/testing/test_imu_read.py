from smbus2 import SMBus
import time

MPU_ADDR = 0x68
bus = SMBus(1)

PWR_MGMT_1 = 0x6B
ACCEL_XOUT_H = 0x3B

bus.write_byte_data(MPU_ADDR, PWR_MGMT_1, 0)

def read_all():
    data = bus.read_i2c_block_data(MPU_ADDR, ACCEL_XOUT_H, 14)

    ax = (data[0] << 8) | data[1]
    ay = (data[2] << 8) | data[3]
    az = (data[4] << 8) | data[5]

    temp = (data[6] << 8) | data[7]

    gx = (data[8] << 8) | data[9]
    gy = (data[10] << 8) | data[11]
    gz = (data[12] << 8) | data[13]

    # convert signed
    def s(v):
        return v - 65536 if v > 32767 else v

    ax, ay, az = s(ax), s(ay), s(az)
    gx, gy, gz = s(gx), s(gy), s(gz)

    temp_c = (temp / 340.0) + 36.53

    return ax, ay, az, gx, gy, gz, temp_c


while True:
    ax, ay, az, gx, gy, gz, temp = read_all()

    print("\n--- IMU ---")
    print("Accel:", ax, ay, az)
    print("Gyro :", gx, gy, gz)
    print("Temp :", temp)

    time.sleep(0.05)  
