#!/usr/bin/env python3

import statistics
import time

from smbus2 import SMBus


ADDR = 0x68
BUS = 1

ACCEL_REG = 0x3B

GYRO_SCALE = 131.0       # LSB/(deg/s)
DEG_TO_RAD = 3.14159265359 / 180.0


def int16(msb, lsb):
    value = (msb << 8) | lsb

    if value & 0x8000:
        value -= 65536

    return value


def read_gyro(bus):
    """
    Read MPU6500 gyroscope.

    Returns:
        gx, gy, gz in rad/s
    """

    data = bus.read_i2c_block_data(
        ADDR,
        ACCEL_REG,
        14
    )

    gx_raw = int16(data[8], data[9])
    gy_raw = int16(data[10], data[11])
    gz_raw = int16(data[12], data[13])


    # Convert raw values -> rad/s

    gx = (gx_raw / GYRO_SCALE) * DEG_TO_RAD
    gy = (gy_raw / GYRO_SCALE) * DEG_TO_RAD
    gz = (gz_raw / GYRO_SCALE) * DEG_TO_RAD


    return gx, gy, gz


def calculate_covariance(samples):

    sigma = statistics.stdev(samples)
    covariance = sigma ** 2

    return sigma, covariance



def main():

    number_of_samples = 2000
    sample_delay = 0.005   # 200 Hz


    gx_samples = []
    gy_samples = []
    gz_samples = []


    print("Starting gyro noise measurement...")
    print("Keep the MPU6500 completely still!")
    time.sleep(2)


    bus = SMBus(BUS)


    # Wake up MPU6500
    bus.write_byte_data(
        ADDR,
        0x6B,
        0x01
    )


    try:

        for i in range(number_of_samples):

            gx, gy, gz = read_gyro(bus)

            gx_samples.append(gx)
            gy_samples.append(gy)
            gz_samples.append(gz)


            time.sleep(sample_delay)


            if i % 100 == 0:
                print(
                    f"Collected {i}/{number_of_samples}"
                )


    finally:
        bus.close()



    print("\nResults:")
    print("----------------------")


    sigma_gx, cov_gx = calculate_covariance(gx_samples)
    sigma_gy, cov_gy = calculate_covariance(gy_samples)
    sigma_gz, cov_gz = calculate_covariance(gz_samples)



    print("Gyro X:")
    print(" sigma =", sigma_gx, "rad/s")
    print(" covariance =", cov_gx, "(rad/s)^2")

    print()


    print("Gyro Y:")
    print(" sigma =", sigma_gy, "rad/s")
    print(" covariance =", cov_gy, "(rad/s)^2")

    print()


    print("Gyro Z:")
    print(" sigma =", sigma_gz, "rad/s")
    print(" covariance =", cov_gz, "(rad/s)^2")



    print("\nROS 2 covariance matrix:")
    print("----------------------")

    print("angular_velocity_covariance = [")
    print(f" {cov_gx}, 0.0, 0.0,")
    print(f" 0.0, {cov_gy}, 0.0,")
    print(f" 0.0, 0.0, {cov_gz}")
    print("]")



if __name__ == "__main__":
    main()