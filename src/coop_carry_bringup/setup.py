from glob import glob
from setuptools import find_packages, setup

setup(
    name='coop_carry_bringup', version='0.2.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/coop_carry_bringup']),
        ('share/coop_carry_bringup', ['package.xml']),
        ('share/coop_carry_bringup/launch', glob('launch/*.launch.py')),
        ('share/coop_carry_bringup/config', glob('config/*.yaml')),
    ],
    install_requires=['setuptools'], tests_require=['pytest'], zip_safe=True,
    entry_points={'console_scripts': [
        'sim_command_adapter = coop_carry_bringup.sim_command_adapter:main',
        'activate_remote_hardware = coop_carry_bringup.activate_remote:main',
        'robot_adapter = coop_carry_bringup.robot_adapter:main',
        'robot_command = coop_carry_bringup.command_client:main',
        'test_independent_arms = coop_carry_bringup.verify_motion:main',
    ]},
    maintainer='Coop Carry Team', maintainer_email='team@example.com',
    description='Distributed dual AI Worker simulation', license='Apache-2.0',
)
