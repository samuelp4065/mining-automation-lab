from setuptools import find_packages, setup

package_name = 'excavator_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Samuel P',
    maintainer_email='samuelp4065@gmail.com',
    description='ArUco detection and depth-based dig-target extraction.',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'aruco_detection_node = excavator_perception.aruco_detection_node:main',
            'proximity_monitor = excavator_perception.proximity_monitor:main',
            'fake_depth = excavator_perception.fake_depth:main',
        ],
    },
)
