from setuptools import setup

package_name = 'afvalrobot'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/' + package_name, ['launch/launch.py'] )
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='tdw',
    maintainer_email='tdw@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sonar = afvalrobot.sonar:main',
            'gripper = afvalrobot.gripper:main',
            'wheels = afvalrobot.wheels:main',
            'camera_processing = afvalrobot.camera_processing:main',
            'state_processor = afvalrobot.state_processor:main',
        ],
    },
)
