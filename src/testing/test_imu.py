from smbus2 import SMBus

bus = SMBus(1)   # I2C bus 1 on Raspberry Pi

MPU_ADDR = 0x68  # or 0x69

try:
    who_am_i = bus.read_byte_data(MPU_ADDR, 0x75)
    print("WHO_AM_I:", hex(who_am_i))
except Exception as e:
    print("Device not responding:", e)
