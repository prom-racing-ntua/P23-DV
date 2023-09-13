from setuptools import setup
import os 
from glob import glob
package_name = 'bench_controller'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')), #extra for launch+config files
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='subdiduntil2',
    maintainer_email='m.serlis.promracing@gmail.com',
    description='TODO: Package description',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyboard_publisher = bench_controller.keyboard_publisher:main', #name for run terminal launch + executable name on launch.py file
            'bench_controller = bench_controller.bench_controller:main'      #reference to main function of .py node
        ],
    },
)

