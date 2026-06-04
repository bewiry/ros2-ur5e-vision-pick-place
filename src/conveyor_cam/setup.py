from setuptools import find_packages, setup

package_name = 'conveyor_cam'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='marwanelqady',
    maintainer_email='marwanelqady@todo.todo',
    description='Conveyor belt computer vision package',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'tracker = conveyor_cam.box_tracker:main',
            'subscriber = conveyor_cam.box_subscriber:main'
        ],
    },
)
