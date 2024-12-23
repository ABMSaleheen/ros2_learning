from setuptools import find_packages, setup
import os
from glob import glob 


package_name = 'rover'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),

        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('lib', package_name), glob('scripts/*')),   
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='saleheen_linux',
    maintainer_email='a.b.m.saleheen@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'obstacle_avoidance = rover.obstacle_avoiding:main'
        ],
    },
)
