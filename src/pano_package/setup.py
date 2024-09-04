from setuptools import setup

package_name = 'pano_package'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='chohs',
    maintainer_email='chohs5133@gmail.com',
    description='TODO: implementing panoramic_image with 2 webcam and opencv',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'panoramic_img_node = pano_package.panoramic_img_node:main',
        ],
    },
)
