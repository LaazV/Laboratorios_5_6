from setuptools import find_packages, setup

package_name = 'turtle_control_lvn'

setup(
    name=package_name,
    version='0.0.1',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='laz',
    maintainer_email='lazaro@email.com',
    description='Laboratório 5',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'turtle_control = turtle_control_lvn.turtle_control:main',
            'turtle_goal = turtle_control_lvn.turtle_goal:main'
        ],
    },
)
