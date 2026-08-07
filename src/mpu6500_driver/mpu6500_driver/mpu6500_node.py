import rclpy
from rclpy.node import Node
from std_srvs.srv import Trigger


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

        # add new publisher
        

        # Initalize bias
        self.gyro_bias_x = 383.98
        self.gyro_bias_y = 182.01
        self.gyro_bias_z = -10.95

        self.accel_bias_x = -882.87
        self.accel_bias_y = -25.29
        self.accel_bias_z = -856.76


        self.get_logger().info(
            'MPU6500 node started, publishing to /imu/data_raw\n'
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
