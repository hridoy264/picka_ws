from setuptools import setup
import os
from glob import glob

package_name = 'picka_description'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, [
            'package.xml',
            'README_GAZEBO.md',
            'VALIDATION_REPORT.md',
        ]),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'meshes'), glob('meshes/*')),
        (os.path.join('share', package_name, 'config'), glob('config/*')),
        (os.path.join('share', package_name, 'worlds'), glob('worlds/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='picka_description maintainers',
    maintainer_email='noreply@example.com',
    description='Gazebo Fortress simulation description and controllers for the Picka arm.',
    license='LicenseRef-Unknown',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
