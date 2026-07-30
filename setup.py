from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'impendence_control'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(where='src', exclude=['test']),
    package_dir={'': 'src'},
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='机器人导纳控制包 - 上位机轨迹规划、导纳控制计算、电流监控',
    license='MIT',
    entry_points={
        'console_scripts': [
            'admittance_control = impendence_control.admittance_calculate:main_admittance_node',
            'trajectory_planner = impendence_control.trajectory_planner:main',
            'current_monitor = impendence_control.current_monitor:main',
        ],
    },
)
