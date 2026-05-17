from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'window_cleaner_planning'

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
    description='Planning layer: boustrophedon coverage planner, deterministic '
                'waypoint follower (default), and Nav2 path follower (advanced mode)',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'coverage_planner = window_cleaner_planning.coverage_planner:main',
            'waypoint_follower = window_cleaner_planning.waypoint_follower:main',
            'path_follower = window_cleaner_planning.path_follower:main',
        ],
    },
)
