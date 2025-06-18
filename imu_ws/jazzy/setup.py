from setuptools import find_packages, setup

package_n = 'imu_mpu6050'

setup(
    name=package_n,
    version='1.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_n]),
        ('share/' + package_n, ['package.xml']),
        ('share/' + package_n + '/launch', ['launch/imu_launch.py']),
        ('share/' + package_n + '/config', ['config/imu_params.yaml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Your Name',
    maintainer_email='your.email@company.com',
    description='ROS2 IMU publisher for MPU6050 sensor with calibration',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'imu_publisher = imu_mpu6050.imu_publisher:main',
        ],
    },
)
