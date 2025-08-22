from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'joy_to_jointstates'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Stanislav Svediroh',
    maintainer_email='stanislav.svediroh@vut.cz',
    description='Package to convert joy msg to manipulator jointstates',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'joy_to_jointstates = joy_to_jointstates.joy_to_jointstates_node:main'
        ],
    },
)
