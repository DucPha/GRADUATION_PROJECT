from setuptools import setup

package_name = 'turn_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        # Include model files if they exist in package share
        ('share/' + package_name + '/models', []),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Pham Minh Duc',
    maintainer_email='duc@todo.todo',
    description='NCNN turn-direction detector for ROS 2 with Vulkan acceleration.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'turn_detector = turn_detector.turn_detector_ncnn:main',
        ],
    },
)
