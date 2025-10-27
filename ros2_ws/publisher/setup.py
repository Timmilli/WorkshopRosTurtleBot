from setuptools import find_packages, setup

import os
from glob import glob

package_name = 'publisher'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        (os.path.join('share', package_name), glob('launch/*launch.[pxy][yma]*')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='student',
    maintainer_email='guenael.mail@gmail.com',
    description='TODO: Package description',
    license='Apache-2.0',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
		'talker1 = publisher.publisher_member_function_q1:main',
		'listener1 = publisher.subscriber_member_function_q1:main',
		'talker2 = publisher.publisher_member_function_q2:main',
		'listener2 = publisher.subscriber_member_function_q2:main',
		'talker3 = publisher.publisher_member_function_q3:main',
		'listener3 = publisher.subscriber_member_function_q3:main',
		'talker4 = publisher.publisher_member_function_q4:main',
		'listener4 = publisher.subscriber_member_function_q4:main',
		'talker5 = publisher.publisher_member_function_q5:main',
		'listener51 = publisher.subscriber_member_function_q5_1:main',
		'listener52 = publisher.subscriber_member_function_q5_2:main',
		'talker6 = publisher.publisher_member_function_q6:main',
		'listener6 = publisher.subscriber_member_function_q6:main',
        ],
    },
)
