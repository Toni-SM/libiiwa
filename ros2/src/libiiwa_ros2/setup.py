from setuptools import setup

package_name = 'libiiwa_ros2'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/default.py', 'launch/test.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Toni-SM',
    maintainer_email='toni@todo.todo',
    description='The libiiwa_ros2 package',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'node = libiiwa_ros2.node:main',
        ],
    },
)
