from setuptools import setup, find_packages
import os
from glob import glob

package_name = 'autonomous_rc_car'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(),  # Automatically finds 'autonomous_rc_car' and 'autonomous_rc_car.nodes'
    data_files=[
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your_email@example.com',
    description='Autonomous indoor RC car package using Lidar and SLAM',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fixed_joint_publisher = autonomous_rc_car.nodes.fixed_joint_publisher:main',
            'fake_odometry_publisher = autonomous_rc_car.nodes.fake_odometry_publisher:main',
            'hybrid_odometry_publisher = autonomous_rc_car.nodes.hybrid_odometry_publisher:main',
        ],
    },
)

