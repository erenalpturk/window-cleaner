from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'window_cleaner_evaluation'

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
        # plot_results.py / run_benchmark.sh are NOT ROS entry points; they
        # are host/dev tools run as `python3 ...` / `bash ...`. Installing
        # them under share/ lets `--symlink-install` hot-link them so edits
        # take effect without a rebuild (same workflow as the launch files).
        # Filter to regular files so a stray scripts/__pycache__ dir (left
        # by a local py_compile/pytest) does not break data_files copying.
        (os.path.join('share', package_name, 'scripts'),
            [f for f in glob('scripts/*') if os.path.isfile(f)]),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Alp Eren Türk',
    maintainer_email='erenalpturk@gmail.com',
    description='Evaluation layer: metrics collection, benchmark runner and result plotting',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'metrics_node = window_cleaner_evaluation.metrics_node:main',
        ],
    },
)
