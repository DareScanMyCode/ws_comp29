from setuptools import find_packages, setup

package_name = 'comp29hardware'

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
    maintainer='cat',
    maintainer_email='529388771@qq.com',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "uwb2ros2 = comp29hardware.uwb2ros2:main",
            "gport2ros2 = comp29hardware.gport2ros2:main",
            "uav2ros2 = comp29hardware.uav2ros2:main"
        ],
    },
)
