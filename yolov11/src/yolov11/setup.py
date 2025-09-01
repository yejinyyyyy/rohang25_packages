from setuptools import setup

package_name = 'yolov11'

setup(
    name=package_name,
    version='0.0.1',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages', ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools',  'torch',
        'torchvision','numpy','opencv-python', 'ultralytics'],
    zip_safe=True,
    maintainer='your_name',
    maintainer_email='your@email.com',
    description='YOLOv11 RTSP Detection Node using ROS 2',
    license='MIT',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'yolov11_node = yolov11.yolov11_node:main',
        ],
    },
)
