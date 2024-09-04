from setuptools import setup
import os
from glob import glob

package_name = 'bev_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name, 'launch'), glob(os.path.join('launch', '*launch.[pxy][yma]*')))
 
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chohs',
    maintainer_email='chohs5133@gmail.com',
    description='TODO: implementing BEV with 2 webcam and opencv',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'bev_node = bev_package.bev_node:main'
        ],
    },
)
