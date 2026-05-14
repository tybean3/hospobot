from setuptools import setup
import os
from glob import glob

package_name = 'hospobot_perception'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob('launch/*.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hospobot',
    maintainer_email='hospobot@todo.todo',
    description='Perception package for the Hospital Delivery Robot',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'object_detector = hospobot_perception.object_detector_node:main'
        ],
    },
)
