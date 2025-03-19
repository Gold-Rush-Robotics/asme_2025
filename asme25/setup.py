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
    maintainer='boozer',
    maintainer_email='boozer@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "actuator_interface = asme25.actuator_interface:main",
            "game_controller = asme25.game_controller:main",
            "computer_vision = asme25.computer_vision:main"
        ],
    },
)
