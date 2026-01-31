import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'robonejopy'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Correção aqui: adicionamos 'share/' + package_name antes do 'launch'
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=[
        'matplotlib',
        'opencv-python-headless',
        'numpy'
    ],
    zip_safe=True,
    maintainer='Vitor',
    maintainer_email='shaftrobotica@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    package_data={'': ['requirements.txt']},
    entry_points={
        'console_scripts': [
            'cameraPub = robonejopy.cameraPub:main',
            'dead_reckoning_node = robonejopy.dead_reckoning_node:main',
            'detectorSQLite = robonejopy.detectorSQLite:main',
            'controleSerial = robonejopy.controleSerial:main',
            'lane_detector_node = robonejopy.lane_detector_node:main',
            'temperature_sensor_node = robonejopy.temperature_sensor_node:main',
            'humidity_node = robonejopy.humidity_node:main',
            'imu_node = robonejopy.imu_node:main',
            'ammonia_sensor_node = robonejopy.ammonia_sensor_node:main',
            'pose_publisher_node = robonejopy.pose_publisher_node:main',
	        'ultrasonic_bridge = robonejopy.ultrasonic_bridge:main',
        ],
    },
    
)
