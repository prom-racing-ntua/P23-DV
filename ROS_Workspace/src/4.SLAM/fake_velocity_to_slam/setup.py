from setuptools import setup
import os
from glob import glob

package_name = 'fake_velocity_to_slam'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'data'), glob('data/*.txt'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vasilis',
    maintainer_email='billvrettos2000@gmail.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'fake_velocity_to_slam = fake_velocity_to_slam.velocity_to_slam:main'
        ],
    },
)
