from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'window_cleaner_perception'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'),
            glob('launch/*.py')),
        (os.path.join('share', package_name, 'config'),
            glob('config/*.yaml')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alp Eren Türk',
    maintainer_email='erenalpturk@gmail.com',
    description='Perception layer: camera, frame detector, dirt segmenter',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node = window_cleaner_perception.camera_node:main',
            'frame_detector = window_cleaner_perception.frame_detector:main',
            'dirt_segmenter = window_cleaner_perception.dirt_segmenter:main',
            'glass_surface_marker = window_cleaner_perception.glass_surface_marker:main',
        ],
    },
)
