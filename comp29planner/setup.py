from setuptools import find_packages, setup
import os, glob
package_name = 'comp29planner'
from setuptools import setup


setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', ['launch/comp29launch_all.py']),
        ('share/' + package_name + '/launch', ['launch/comp29test_form.py']),
        ('share/' + package_name + '/launch', ['launch/comp29test_fsm.py']),
        ('share/' + package_name + '/launch', ['launch/comp29test_form_leaving.py']),
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
            "test_rend = comp29planner.test_rend:main",
            "test_formation = comp29planner.test_formation:main",
            "comp29main = comp29planner.comp29planner_main:main"
        ],
    },
)
