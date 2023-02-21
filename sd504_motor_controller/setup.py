from setuptools import setup

package_name = 'sd504_motor_controller'

setup(
    name=package_name,
    version='1.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Jonathan T. Boylan',
    maintainer_email='jboylan@fsu.edu',
    description='Team 504 Senior Design Motor Controller Package',
    license='BSD',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'motor_controller_node = sd504_motor_controller.motor_controller_node:main'
        ],
    },
)
