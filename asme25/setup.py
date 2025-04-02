from setuptools import find_packages, setup

package_name = 'asme25'

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
    maintainer='Gold Rush Robotics',
    maintainer_email='goldrushrobotics@gmail.com',
    description='Contains Nodes for the 2024-2025 ASME EFx SDC Robot',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "actuator_interface = asme25.actuator_interface:main",
            "game_controller = asme25.game_controller:main",
            "computer_vision = asme25.computer_vision:main"
        ],
    },
)
