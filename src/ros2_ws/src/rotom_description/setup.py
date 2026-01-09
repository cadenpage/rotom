from setuptools import find_packages, setup
import os
from glob import glob

package_name = 'rotom_description'

setup(
    name=package_name,
    version='0.0.0',
    packages=find_packages(exclude=['test']),
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']), 
        (os.path.join('share', 'rotom_description', 'launch'), glob('launch/*.py')),
        (os.path.join('share', 'rotom_description', 'urdf'), glob('urdf/*')),
        (os.path.join('share', package_name, 'rviz'), glob('rviz/*')),
        (os.path.join('share', package_name, 'meshes', 'visual'), glob('meshes/visual/*')),
        (os.path.join('share', package_name, 'meshes', 'collision'), glob('meshes/collision/*')),

    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='caden',
    maintainer_email='ckp634@my.utexas.edu',
    description='TODO: Package description',
    license='TODO: License declaration',
    extras_require={
        'test': [
            'pytest',
        ],
    },
    entry_points={
        'console_scripts': [
        ],
    },
)
