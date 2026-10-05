from setuptools import setup

package_name = 'gui_matplotlib'

setup(
    name=package_name,
    version='0.1.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
         ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/gui_matplotlib.launch.py']),
    ],
    install_requires=['setuptools'],
    zip_safe=False,
    maintainer='Pham Minh Duc',
    maintainer_email='pmd0793724602@gmail.com',
    description='Python + Matplotlib white-theme dashboard with polar LIDAR map',
    license='MIT',
    entry_points={
        'console_scripts': [
            'gui_matplotlib = gui_matplotlib.dashboard_matplotlib:main',
        ],
    },
)
