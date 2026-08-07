#!/usr/bin/env python3

import statistics
import time

from smbus2 import SMBus


ADDR = 0x68
BUS = 1

ACCEL_REG = 0x3B

ACCEL_SCALE = 16384.0   # LSB/g
G = 9.80665


def int16(msb, lsb):
    value = (msb << 8) | lsb

    if value & 0x8000:
        value -= 65536

    return value



def read_accel(bus):
    """
    Read MPU6500 accelerometer.

    Returns:
        ax, ay, az in m/s^2
    """

    data = bus.read_i2c_block_data(
        ADDR,
        ACCEL_REG,
        14
    )

    ax_raw = int16(data[0], data[1])
    ay_raw = int16(data[2], data[3])
    az_raw = int16(data[4], data[5])


    ax = (ax_raw / ACCEL_SCALE) * G
    ay = (ay_raw / ACCEL_SCALE) * G
    az = (az_raw / ACCEL_SCALE) * G


    return ax, ay, az



def calculate_covariance(samples):

    sigma = statistics.stdev(samples)
    covariance = sigma ** 2

    return sigma, covariance



def remove_bias(samples):

    mean = statistics.mean(samples)

    return [
        x - mean
        for x in samples
    ]



def main():

    number_of_samples = 2000
    sample_delay = 0.005


    ax_samples = []
    ay_samples = []
    az_samples = []


    print("Starting accelerometer noise measurement...")
    print("Keep MPU6500 completely still!")
    print("Place it in a fixed orientation.")
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

            ax, ay, az = read_accel(bus)

            ax_samples.append(ax)
            ay_samples.append(ay)
            az_samples.append(az)


            time.sleep(sample_delay)


            if i % 100 == 0:
                print(
                    f"Collected {i}/{number_of_samples}"
                )


    finally:
        bus.close()



    # Remove gravity and bias
    ax_noise = remove_bias(ax_samples)
    ay_noise = remove_bias(ay_samples)
    az_noise = remove_bias(az_samples)



    sigma_x, cov_x = calculate_covariance(ax_noise)
    sigma_y, cov_y = calculate_covariance(ay_noise)
    sigma_z, cov_z = calculate_covariance(az_noise)



    print("\nResults:")
    print("----------------------")


    print("Accel X:")
    print(" sigma =", sigma_x, "m/s^2")
    print(" covariance =", cov_x, "(m/s^2)^2")

    print()


    print("Accel Y:")
    print(" sigma =", sigma_y, "m/s^2")
    print(" covariance =", cov_y, "(m/s^2)^2")

    print()


    print("Accel Z:")
    print(" sigma =", sigma_z, "m/s^2")
    print(" covariance =", cov_z, "(m/s^2)^2")



    print("\nROS 2 covariance matrix:")
    print("----------------------")

    print("linear_acceleration_covariance = [")
    print(f" {cov_x}, 0.0, 0.0,")
    print(f" 0.0, {cov_y}, 0.0,")
    print(f" 0.0, 0.0, {cov_z}")
    print("]")



if __name__ == "__main__":
    main()