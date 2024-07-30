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
            'conducao = robonejopy.conducao:main',
            'camCheck = robonejopy.camCheck:main',
            'cameraPub = robonejopy.cameraPub:main',
            'navegacao = robonejopy.navegacao:main',
            'detector = robonejopy.detector:main',
            'detectorCheck = robonejopy.detectorCheck:main',
            'controleSerial = robonejopy.controleSerial:main',
            'seguidorCorredor = robonejopy.seguidorCorredor:main',
            'lane_detector_node = robonejopy.lane_detector_node:main',
            'temperature_sensor_node = robonejopy.temperature_sensor_node:main',
            'ambient_temperature_node = robonejopy.ambient_temperature_node:main'
        ],
    },
    
)
