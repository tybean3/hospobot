from setuptools import find_packages, setup

package_name = 'odesc_hardware'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools', 'pyserial'],
    zip_safe=True,
    maintainer='user',
    maintainer_email='user@todo.todo',
    description='ROS 2 hardware interface for ODESC differential drive',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'odesc_drive_node = odesc_hardware.odesc_drive_node:main',
            'cmd_vel_mux = odesc_hardware.cmd_vel_mux:main',
            'diagnostics_node = odesc_hardware.diagnostics_node:main',
            'nav2_manager_node = odesc_hardware.nav2_manager_node:main',
            'bno086_node = odesc_hardware.bno086_node:main',
        ],
    },
)
