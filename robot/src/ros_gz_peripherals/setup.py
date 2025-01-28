from setuptools import find_packages, setup

package_name = 'ros_gz_peripherals'

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
    maintainer='kevin',
    maintainer_email='kevin@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'led_controller = ros_gz_peripherals.led_controller_node:main', 
            'sound_controller = ros_gz_peripherals.sound_controller_node:main', 
        ],
    },
)
