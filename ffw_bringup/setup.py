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

setup(
    name=package_name,
    version='2.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name + '/launch', glob('launch/*.launch.py')),
        (os.path.join('share', package_name, 'config/ffw_bg2_rev2_follower'),
         glob('config/ffw_bg2_rev2_follower/*')),
        (os.path.join('share', package_name, 'config/ffw_bg2_rev3_follower'),
         glob('config/ffw_bg2_rev3_follower/*')),
        (os.path.join('share', package_name, 'config/ffw_bg2_rev4_follower'),
         glob('config/ffw_bg2_rev4_follower/*')),
        (os.path.join('share', package_name, 'config/ffw_bh5_rev1_follower'),
         glob('config/ffw_bh5_rev1_follower/*')),
        (os.path.join('share', package_name, 'config/ffw_f2_follower'),
         glob('config/ffw_f2_follower/*')),
        (os.path.join('share', package_name, 'config/ffw_sg2_rev1_follower'),
         glob('config/ffw_sg2_rev1_follower/*')),
        (os.path.join('share', package_name, 'config/ffw_lg2_leader'),
         glob('config/ffw_lg2_leader/*')),
        (os.path.join('share', package_name, 'config/ffw_sh5_rev1_follower'),
         glob('config/ffw_sh5_rev1_follower/*')),
        (os.path.join('share', package_name, 'config/common'), glob('config/common/*')),
        ('share/' + package_name + '/worlds', glob('worlds/*.sdf')),
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
            'head_eef_tracker = ffw_bringup.head_eef_tracker:main',
            'link_pose_logger = ffw_bringup.link_pose_logger:main',
            'vision_pick_place = ffw_bringup.vision_pick_place:main',
            'color_bottle_pick_place = ffw_bringup.color_bottle_pick_place:main',
            'bottle_basket_pick_place = ffw_bringup.bottle_basket_pick_place:main',
            'bgra_to_bgr_compressed = ffw_bringup.bgra_to_bgr_compressed:main',
            'centerpose_bottle_pick = ffw_bringup.centerpose_bottle_pick:main',
            'bottle_box = ffw_bringup.bottle_box:main',
            'pointcloud_crop = ffw_bringup.pointcloud_crop:main',
            'left_gripper_controller = ffw_bringup.left_gripper_controller:main',
            'image_plane_pointcloud = ffw_bringup.image_plane_pointcloud:main',
            'table_plane_pointcloud = ffw_bringup.table_plane_pointcloud:main',
            'depth_to_pointcloud = ffw_bringup.depth_to_pointcloud:main',
        ],
    },
)
