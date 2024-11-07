from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'prius_line_following'

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
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*.urdf')),
        # (os.path.join('lib', package_name), glob('scripts/*')),   

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
            'video_saver = prius_line_following.video_save:main',
            'prius_line_follower = prius_line_following.line_following:main'

        ],
    },
)
