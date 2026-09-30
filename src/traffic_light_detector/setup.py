from setuptools import setup

package_name = 'traffic_light_detector'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Pham Minh Duc',
    maintainer_email='duc@todo.todo',
    description='NCNN traffic-light and sign detector for ROS 2 with Vulkan acceleration.',
    license='MIT',
    entry_points={
        'console_scripts': [
            'traffic_light_detector = traffic_light_detector.traffic_light_ncnn:main',
        ],
    },
)
