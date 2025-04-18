import glob

from setuptools import find_packages, setup

package_name = 'ffw_bringup'
authors_info = [
    ('Sungho Woo', 'wsh@robotis.com'),
    ('Woojin Wie', 'wwj@robotis.com'),
    ('Wonho Yun', 'ywh@robotis.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)

setup(
    name=package_name,
    version='1.0.2',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['ffw.rules']),
        ('share/' + package_name + '/launch', glob.glob('launch/*.launch.py')),
        ('share/' + package_name + '/config', glob.glob('config/*.yaml')),
        ('share/' + package_name + '/worlds', glob.glob('worlds/*.sdf')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=authors_info,
    author_email=author_emails,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    keywords=['ROS'],
    classifiers=[
        'Intended Audience :: Developers',
        'License :: OSI Approved :: Apache Software License',
        'Programming Language :: Python',
        'Topic :: Software Development',
    ],
    description='ROS 2 launch scripts for starting the FFW',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'hand_controller_left = ffw_bringup.hand_controller_left:main',
            'hand_controller_right = ffw_bringup.hand_controller_right:main',
            'hand_calibrator_left = ffw_bringup.hand_calibrator_left:main',
            'hand_calibrator_right = ffw_bringup.hand_calibrator_right:main',
            'hand_controller_setting = ffw_bringup.hand_controller_setting:main',
            'init_position = ffw_bringup.init_position:main',
            'ffw_create_udev_rules = ffw_bringup.ffw_create_udev_rules:main',
        ],
    },
)
