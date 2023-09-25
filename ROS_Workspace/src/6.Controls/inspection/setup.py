from setuptools import setup
from glob import glob
import os
package_name = 'inspection'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='dimitris',
    maintainer_email='d.papailiopoulos.promracing@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'run = inspection.dummy_inspection:main',
            'pc_error = inspection.pc_error:main',
            'insp = inspection.lifecycle_inspection:main',
            'keyboard_publisher = inspection.keyboard_publisher:main'
        ],
    },
)