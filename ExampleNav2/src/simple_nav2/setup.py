import os  
from glob import glob
from setuptools import setup

package_name = 'simple_nav2'

setup(
 name=package_name,
 version='0.0.0',
 packages=[package_name],
 data_files=[
     ('share/ament_index/resource_index/packages',
             ['resource/' + package_name]),
     ('share/' + package_name, ['package.xml']),
     (os.path.join('share', package_name, 'launch'), glob('launch/*.[pxy][yma]*')),
     (os.path.join('share', package_name, 'config'), glob('config/*.[ry]*')),
     (os.path.join('share', package_name, 'map'), glob('map/*.[py]*')),
     (os.path.join('share', package_name, 'param'), glob('param/*.[y]*')),
   ],
 install_requires=['setuptools'],
 zip_safe=True,
 maintainer='TODO',
 maintainer_email='TODO',
 description='TODO: Package description',
 license='TODO: License declaration',
 tests_require=['pytest'],
 entry_points={
     'console_scripts': [
             'simple_nav2 = simple_nav2.sim_nav_goals:main'
     ],
   },
)