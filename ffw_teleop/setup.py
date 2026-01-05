from setuptools import find_packages, setup

package_name = 'ffw_teleop'
authors_info = [
    ('Sungho Woo', 'wsh@robotis.com'),
    ('Woojin Wie', 'wwj@robotis.com'),
    ('Wonho Yun', 'ywh@robotis.com'),
]
authors = ', '.join(author for author, _ in authors_info)
author_emails = ', '.join(email for _, email in authors_info)
setup(
    name=package_name,
    version='1.1.16',
    packages=find_packages(exclude=[]),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    author=authors,
    author_email=author_emails,
    maintainer='Pyo',
    maintainer_email='pyo@robotis.com',
    description='FFW teleop ROS 2 package.',
    license='Apache 2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'keyborad_control = ffw_teleop.keyboard_control:main',
        ],
    },
)
