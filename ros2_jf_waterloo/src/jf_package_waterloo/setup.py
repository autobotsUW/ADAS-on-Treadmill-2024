from setuptools import find_packages, setup

package_name = 'jf_package_waterloo'

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
    maintainer='jf',
    maintainer_email='jfarnault29@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'camera_node=jf_package_waterloo.camera_node:main',
            'input_node=jf_package_waterloo.input_node:main',
            'control_node=jf_package_waterloo.control_node:main',
            'bluetooth_node=jf_package_waterloo.bluetooth_node:main',
            'display_node=jf_package_waterloo.display_node:main',
            'avoiding_obstacles_node=jf_package_waterloo.avoiding_obstacles_node:main',
            'treadmill_control_node=jf_package_waterloo.treadmill_control_node:main',
            'security_node=jf_package_waterloo.security_node:main',
            'treadmill_interface=jf_package_waterloo.treadmill_interface:main',
            
        ],
    },
)
