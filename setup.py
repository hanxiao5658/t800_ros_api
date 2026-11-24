from setuptools import setup

package_name = 't800_ros_api'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='T800 development team',
    maintainer_email='maintainer@example.invalid',
    description='ROS 2 topic and service REST gateway for T800',
    license='MIT',
    entry_points={
        'console_scripts': [
            'api_server = t800_ros_api.server:main',
        ],
    },
)
