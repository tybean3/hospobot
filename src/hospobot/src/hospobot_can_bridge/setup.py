from setuptools import find_packages, setup

package_name = 'hospobot_can_bridge'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='hospobot',
    maintainer_email='tybean3@github.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
            'system_can_bridge = hospobot_can_bridge.system_can_bridge:main',
            'odrive_can_node = hospobot_can_bridge.odrive_can_node:main',
            'odom_republisher = hospobot_can_bridge.odom_republisher:main',
        ],
    },
)
