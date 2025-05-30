import os
from glob import glob
from setuptools import find_packages, setup

package_name = 'suave_rosa_runner'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name,
            'launch'), glob('launch/*launch.[pxy][yma]*')),
        (os.path.join('share', package_name,
            'config'), glob('config/*'))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Gustavo Rezende',
    maintainer_email='g.rezendesilva@tudelft.nl',
    description='Package with runner config for ROSA + SUAVE',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
        ],
    },
)
