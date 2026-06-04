from setuptools import setup

package_name = 'cpsl_detection'

setup(
    name=package_name,
    version='0.1.0',
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
    description='YOLO object detection with 3D localization in the SLAM map',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yoloDetector = cpsl_detection.yolo_detector:main',
        ],
    },
)
