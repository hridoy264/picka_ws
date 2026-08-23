from setuptools import find_packages, setup

package_name = 'picka_hardware'

setup(
    name=package_name,
    version='0.1.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/config', ['config/hardware.yaml']),
        ('share/' + package_name + '/launch', ['launch/hardware.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Hridoy',
    maintainer_email='hridoy@example.com',
    description='USB serial bridge for the Picka ESP32 controller.',
    license='Apache-2.0',
    entry_points={'console_scripts': ['serial_bridge = picka_hardware.serial_bridge:main']},
)
