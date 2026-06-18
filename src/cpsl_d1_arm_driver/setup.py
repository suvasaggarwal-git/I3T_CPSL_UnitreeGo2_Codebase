from setuptools import setup

package_name = 'cpsl_d1_arm_driver'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='cpsl',
    maintainer_email='suvas.aggarwal@duke.edu',
    description='Unitree D1 Arm ROS2 driver',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'd1_arm_node = cpsl_d1_arm_driver.d1_arm_node:main'
        ],
    },
)
