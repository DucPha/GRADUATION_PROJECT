from setuptools import setup

package_name = 'turn_detector'

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
    maintainer='Duc Pham',
    maintainer_email='duc@todo.todo',
    description='NCNN turn-direction detector for ROS 2.',
    license='TODO',
    entry_points={
        'console_scripts': [
            'turn_detector = turn_detector.turn_detector_ncnn:main',
        ],
    },
)
