from setuptools import find_packages
from setuptools import setup

setup(
    name='yolov11_msgs',
    version='0.0.1',
    packages=find_packages(
        include=('yolov11_msgs', 'yolov11_msgs.*')),
)
