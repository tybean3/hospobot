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
        (os.path.join('share', package_name, 'config'), glob('config/*.yaml')),
        (os.path.join('share', package_name, 'config', 'images'), glob('config/images/*')),
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
            'object_detector = hospobot_perception.object_detector_node:main',
            'oakd_yolo_node = hospobot_perception.oakd_yolo_node:main',
            'semantic_projection_node = hospobot_perception.semantic_projection_node:main',
            'semantic_tracker_node = hospobot_perception.semantic_tracker_node:main',
            'semantic_costmap_bridge = hospobot_perception.semantic_costmap_bridge:main'
        ],
    },
)
