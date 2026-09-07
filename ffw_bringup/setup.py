from glob import glob
import os

from setuptools import find_packages, setup

package_name = 'ffw_bringup'
authors_info = [
    ('Sungho Woo', 'wsh@robotis.com'),
    ('Woojin Wie', 'wwj@robotis.com'),
    ('Wonho Yun', 'ywh@robotis.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)
controller_configs = [
    (os.path.join('share', package_name, directory),
     glob(os.path.join(directory, '*.yaml')))
    for directory in glob('config/follower/controllers/*')
]

setup(
    name=package_name,
    version='2.2.5',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config/ffw_lg2_leader'),
         glob('config/ffw_lg2_leader/*')),
        (os.path.join('share', package_name, 'config/ffw_lg2_rev2_leader'),
         glob('config/ffw_lg2_rev2_leader/*')),
        (os.path.join('share', package_name, 'config/ffw_lg2_mini_leader'),
         glob('config/ffw_lg2_mini_leader/*')),
        (os.path.join('share', package_name, 'config/common'), glob('config/common/*')),
        (os.path.join('share', package_name, 'config/follower'),
         glob('config/follower/*.yaml')),
    ] + controller_configs + [
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
        (os.path.join('share', package_name, 'config/ffw_a3'),
         glob('config/ffw_a3/*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=authors,
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
            'joint_trajectory_executor = ffw_bringup.joint_trajectory_executor:main',
            'joint_trajectory_splitter = ffw_bringup.joint_trajectory_splitter:main',
            'head_eef_tracker = ffw_bringup.head_eef_tracker:main',
            'finish_monitor = ffw_bringup.finish_monitor:main',
            'foot_switch_node = ffw_bringup.foot_switch:main',
        ],
    },
)
