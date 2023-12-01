from setuptools import setup

package_name = 'node_logger'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, package_name + '.node_logger'],
    install_requires=['setuptools'],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    zip_safe=True,
    author='nick',
    author_email='n.adamopoulos.promracing@gmail.com',
    maintainer='nick',
    maintainer_email='n.adamopoulos.promracing@gmail.com',
    description='Master Library for timestamp logging across workspace',
    license='Apache License 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)