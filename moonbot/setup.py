from setuptools import setup

package_name = 'moonbot'
subfolders = "moonbot.utilities"
setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name, subfolders],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='Aditya',
    maintainer_email='aditya.prakash.r4@dc.tohoku.ac.jp',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            "dynamixel_control = moonbot.dynamixel_control:main",
            "joint_controller = moonbot.joint_controller:main"
        ],
    },
)
