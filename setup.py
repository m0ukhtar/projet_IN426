from setuptools import setup
import os
from glob import glob

package_name = 'in426_motion'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join("share/" + package_name), glob("launch/*_launch.py"))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Johvany Gustave',
    maintainer_email='johvany.gustave@ipsa.fr',
    description='This package contains the scripts that send the adequation commands to the simulated robots',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "send_goal_test.py = in426_motion.send_goal_test:main",
            "ik_1.py = in426_motion.ik_1:main",
            "motion_robot_2.py = in426_motion.motion_robot_2:main"
        ],
    },
)
