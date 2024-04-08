from setuptools import find_packages, setup

package_name = 'triple_axis_servo_controller_py'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    py_modules=['serialcomms'],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='splot',
    maintainer_email='wwilliamcook@gmail.com',
    description='Node for communicating with TripleAxisServoController',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'triple_axis_servo_controller = triple_axis_servo_controller_py.triple_axis_servo_controller:main'
        ],
    },
)
