from setuptools import find_packages, setup

package_name = 'picka_kinematics'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    package_data={'': ['py.typed']},
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='macbookair',
    maintainer_email='macbookair@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'fk_server = picka_kinematics.fk_server:main',
            'picka_cli = picka_kinematics.picka_cli:main',
            'ik_server = picka_kinematics.ik_server:main',
            'serial_bridge_node = picka_kinematics.serial_bridge_node:main',
        ],
    },
)
