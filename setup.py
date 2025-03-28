from setuptools import find_packages, setup
import os
import glob

package_name = 'leap_sim'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        ('share/leap_sim/leap_hand_mesh_right', glob.glob('resource/leap_hand_mesh_right/*')),
        (os.path.join('share', package_name, 'launch'), glob.glob(os.path.join('launch', 'launch*.[pxy][yma]*')))
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='lars',
    maintainer_email='lars@todo.todo',
    description='TODO: Package description',
    license='Apache-2.0',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'leap_sim = leap_sim.leap_sim:main'
            
        ],
    },
)
