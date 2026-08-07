import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger
from ahrs.filters import Madgwick 
import numpy as np

from sensor_msgs.msg import Imu

from smbus2 import SMBus
import time


ADDR = 0x68
BUS = 1

ACCEL_REG = 0x3B

ACCEL_SCALE = 16384.0
GYRO_SCALE = 131.0

G = 9.80665


def int16(msb, lsb):
    value = (msb << 8) | lsb
    if value & 0x8000:
        value -= 65536
    return value


class MPU6500(Node):

    def __init__(self):

        super().__init__('mpu6500')

        self.publisher = self.create_publisher(
            Imu,
            '/imu/data_raw',
            100
        )

        # Filtered IMU publisher with orientation
        self.filtered_publisher = self.create_publisher(
            Imu,
            '/imu/data',
            100
        )

        # Initialize Madgwick AHRS filter
        # Frequency: 100 Hz (matching timer period of 0.01s)
        # Gain: 0.033 (default for IMU implementations)
        self.madgwick = Madgwick(frequency=100.0, gain=0.033)
        
        # Initial quaternion (identity: no rotation)
        self.quaternion = np.array([1.0, 0.0, 0.0, 0.0])

        # Service
        self.reset_srv = self.create_service(
            Trigger,
            '/imu/calibrate_gyro',
            self.calibration_callback
        )


        self.reset_srv = self.create_service(
            Trigger,
            '/imu/calibrate_accel',
            self.calibration_callback_accel
        )
        
        
        self.bus = SMBus(BUS)

        # wake up
        self.bus.write_byte_data(
            ADDR,
            0x6B,
            0x01
        )

        self.timer = self.create_timer(
            0.01,
            self.publish_imu 
        )

        # Initalize bias
        self.gyro_bias_x = 383.98
        self.gyro_bias_y = 182.01
        self.gyro_bias_z = -10.95

        self.accel_bias_x = -882.87
        self.accel_bias_y = -25.29
        self.accel_bias_z = -856.76


        self.get_logger().info(
            'MPU6500 node started, publishing to /imu/data_raw\n'
            'Filtered IMU with orientation published to /imu/data\n'
            'Calibrate gyroscope using /imu/calibrate_gyro\n'
            'Calibrate accelerometer using /imu/calibrate_accel\n'
        )
        
    def publish_imu(self):

        data = self.bus.read_i2c_block_data(
            ADDR,
            ACCEL_REG,
            14
        )

        ax = int16(data[0], data[1])
        ay = int16(data[2], data[3])
        az = int16(data[4], data[5])

        gx = int16(data[8], data[9])
        gy = int16(data[10], data[11])
        gz = int16(data[12], data[13])


        msg = Imu()

        msg.header.stamp = self.get_clock().now().to_msg()

        msg.header.frame_id = "imu_link"


        # gyro: degrees/sec -> radians/sec

        msg.angular_velocity.x = (
            (gx- self.gyro_bias_x) / GYRO_SCALE
        ) * 3.141592 / 180.0

        msg.angular_velocity.y = (
            (gy- self.gyro_bias_y) / GYRO_SCALE
        ) * 3.141592 / 180.0

        msg.angular_velocity.z = (
            (gz - self.gyro_bias_z) / GYRO_SCALE
        ) * 3.141592 / 180.0

        msg.angular_velocity_covariance = [
            5.686050123062371e-06, 0.0, 0.0,
            0.0, 5.851967223857973e-06, 0.0,
            0.0, 0.0, 7.3749893676282255e-06
        ]
        
        # acceleration

        msg.linear_acceleration.x = (
            (ax - self.accel_bias_x) / ACCEL_SCALE
        ) * G

        msg.linear_acceleration.y = (
            (ay - self.accel_bias_y) / ACCEL_SCALE
        ) * G

        msg.linear_acceleration.z = (
            (az - self.accel_bias_z) / ACCEL_SCALE
        ) * G

        msg.linear_acceleration_covariance = [
            0.0004555583280514468, 0.0, 0.0,
            0.0, 0.0004419660765857735, 0.0,
            0.0, 0.0, 0.0013774022818390963
        ]

        # No orientation yet
        msg.orientation_covariance[0] = -1


        self.publisher.publish(msg)

        # Publish filtered IMU data with orientation from Madgwick filter
        self.publish_filtered_imu(ax, ay, az, gx, gy, gz, msg.header.stamp)

    def publish_filtered_imu(self, ax_raw, ay_raw, az_raw, gx_raw, gy_raw, gz_raw, stamp):
        """
        Publish filtered IMU data with orientation estimated by Madgwick AHRS filter.
        
        Args:
            ax_raw, ay_raw, az_raw: Raw accelerometer values (LSB)
            gx_raw, gy_raw, gz_raw: Raw gyroscope values (LSB)
            stamp: ROS2 timestamp for message header
        """
        # Convert to physical units as numpy arrays
        accel_mps2 = np.array([
            ((ax_raw - self.accel_bias_x) / ACCEL_SCALE) * G,
            ((ay_raw - self.accel_bias_y) / ACCEL_SCALE) * G,
            ((az_raw - self.accel_bias_z) / ACCEL_SCALE) * G
        ])
        
        gyro_radps = np.array([
            ((gx_raw - self.gyro_bias_x) / GYRO_SCALE) * 3.141592 / 180.0,
            ((gy_raw - self.gyro_bias_y) / GYRO_SCALE) * 3.141592 / 180.0,
            ((gz_raw - self.gyro_bias_z) / GYRO_SCALE) * 3.141592 / 180.0
        ])
        
        # Update Madgwick filter with gyroscope (rad/s) and accelerometer (m/s^2)
        # updateIMU requires: previous_quaternion, gyr_array, acc_array
        self.quaternion = self.madgwick.updateIMU(self.quaternion, gyr=gyro_radps, acc=accel_mps2)
        
        # Create filtered IMU message
        filtered_msg = Imu()
        filtered_msg.header.stamp = stamp
        filtered_msg.header.frame_id = "imu_link"
        
        # Orientation from Madgwick filter
        filtered_msg.orientation.w = float(self.quaternion[0])
        filtered_msg.orientation.x = float(self.quaternion[1])
        filtered_msg.orientation.y = float(self.quaternion[2])
        filtered_msg.orientation.z = float(self.quaternion[3])
        
        # Orientation covariance (estimated based on filter performance)
        # Typical values for Madgwick with good tuning
        filtered_msg.orientation_covariance = [
            0.01, 0.0, 0.0,
            0.0, 0.01, 0.0,
            0.0, 0.0, 0.01
        ]
        
        # Angular velocity (same as raw, already bias-corrected)
        filtered_msg.angular_velocity.x = gyro_radps[0]
        filtered_msg.angular_velocity.y = gyro_radps[1]
        filtered_msg.angular_velocity.z = gyro_radps[2]
        filtered_msg.angular_velocity_covariance = [
            5.686050123062371e-06, 0.0, 0.0,
            0.0, 5.851967223857973e-06, 0.0,
            0.0, 0.0, 7.3749893676282255e-06
        ]
        
        # Linear acceleration (same as raw, already bias-corrected)
        filtered_msg.linear_acceleration.x = accel_mps2[0]
        filtered_msg.linear_acceleration.y = accel_mps2[1]
        filtered_msg.linear_acceleration.z = accel_mps2[2]
        filtered_msg.linear_acceleration_covariance = [
            0.0004555583280514468, 0.0, 0.0,
            0.0, 0.0004419660765857735, 0.0,
            0.0, 0.0, 0.0013774022818390963
        ]
        
        self.filtered_publisher.publish(filtered_msg)

    def calibration_callback(self, request, response):
        self.get_logger().info(
            "Starting Gyro Calibration..."
        )

        self.calibrate_gyro()


        response.success = True
        response.message = "IMU calibration completed"

        return response
        

    def calibration_callback_accel(self, request, response):
        self.get_logger().info(
            "Starting Accel Calibration..."
        )
        self.get_logger().info(
            "Put sensor flat on ground, z axis pointing up"
        )
        
        self.calibrate_accel()


        response.success = True
        response.message = "IMU accel calibration completed"

        return response


    def calibrate_gyro(self, samples=1000):

        gx_sum = 0
        gy_sum = 0
        gz_sum = 0
        
        self.get_logger().info(
            "Calibrating gyro. Keep IMU still..."
        )

        for _ in range(samples):

            data = self.bus.read_i2c_block_data(
                ADDR,
                ACCEL_REG,
                14
            )
            
            gx_sum += int16(data[8], data[9])
            gy_sum += int16(data[10], data[11])
            gz_sum += int16(data[12], data[13])

            time.sleep(0.002)

        self.gyro_bias_x = gx_sum / samples
        self.gyro_bias_y = gy_sum / samples
        self.gyro_bias_z = gz_sum / samples

        self.get_logger().info(
            f"Gyro bias: "
            f"{self.gyro_bias_x:.2f}, "
            f"{self.gyro_bias_y:.2f}, "
            f"{self.gyro_bias_z:.2f}"
        )


    def calibrate_accel(self, samples=1000):
 
        ax_sum = 0
        ay_sum = 0
        az_sum = 0

        self.get_logger().info(
            "Calibrating accel. Keep IMU still..."
        )

        for _ in range(samples):

            data = self.bus.read_i2c_block_data(
                ADDR,
                ACCEL_REG,
                14
            )
            
            ax_sum += int16(data[0], data[1])
            ay_sum += int16(data[2], data[3])
            az_sum += int16(data[4], data[5])

            time.sleep(0.002)
            
        self.accel_bias_x = ax_sum / samples
        self.accel_bias_y = ay_sum / samples
        self.accel_bias_z = (az_sum / samples) - ACCEL_SCALE


        self.get_logger().info(
            f"Accel bias: "
            f"{self.accel_bias_x:.2f}, "
            f"{self.accel_bias_y:.2f}, "
            f"{self.accel_bias_z:.2f}"
        )

def main():

    rclpy.init()

    node = MPU6500()

    rclpy.spin(node)

    node.destroy_node()

    rclpy.shutdown()


if __name__ == '__main__':
    main()
