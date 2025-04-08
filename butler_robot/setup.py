from setuptools import find_packages, setup

package_name = 'butler_robot'

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
    maintainer='kapil',
    maintainer_email='kapil@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "butler_navigation= butler_robot.butler_navigation:main ",
            "food_placement_confirmation_server= butler_robot.food_placement_confirmation_server:main",
            "main_controller= butler_robot.main_controller:main",
        ],
    },
)
