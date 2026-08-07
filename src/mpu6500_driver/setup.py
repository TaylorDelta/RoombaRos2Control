from setuptools import find_packages, setup

package_name = 'mpu6500_driver'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=[
        'setuptools',
        'smbus2',
        'ahrs',
    ],
    zip_safe=True,
    maintainer='raspitwo',
    maintainer_email='raspitwo@todo.todo',
    description='ROS2 driver for MPU6500 IMU',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'mpu6500_node = mpu6500_driver.mpu6500_node:main',
		'mpu6500_filtered_node = mpu6500_driver.mpu6500_filtered_node:main',
        ],
    },
)
