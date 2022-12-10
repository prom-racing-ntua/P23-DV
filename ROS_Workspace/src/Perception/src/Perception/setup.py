from setuptools import setup
import os
from glob import glob

package_name = 'Perception'
libraries = 'Perception/libraries'
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, libraries],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'numpyObjects'), glob('numpyObjects/*.npy')),
        (os.path.join('share', package_name, 'models'), glob('models/*.pt'))
    ],

    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='vasilis',
    maintainer_email='billvrettos2000@gmail.com',
    description='Prom Racing P23 Perception Package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'acquisition = Perception.acquisitionNode:main',
            'acquisition_logger = Perception.acquisitionLoggerNode:main',
            'inference = Perception.inferenceNode:main'
        ],
    },
)
