from setuptools import setup
import os               # <--- 1. 新增这行
from glob import glob   # <--- 2. 新增这行

package_name = 'wall_robot_pkg'

setup(
    name=package_name,
    version='0.0.0',
    packages=[package_name],
    data_files=[
        ('share/ament_index/resource_index/packages',
            ['resource/' + package_name]),
        ('share/' + package_name, ['package.xml']),
        
        # === 3. 新增下面这一行 (最重要的一步) ===
        (os.path.join('share', package_name, 'launch'), glob('launch/*.launch.py')),
    ],
    install_requires=['setuptools'],
    zip_safe=True,
    maintainer='c403',
    maintainer_email='c403@todo.todo',
    description='TODO: Package description',
    license='TODO: License declaration',
    tests_require=['pytest'],
    entry_points={
        'console_scripts': [
            'sick_odom_node = wall_robot_pkg.sick_odom_node:main',
            'mission_controller_node = wall_robot_pkg.mission_controller_node:main',
            'camera_node = wall_robot_pkg.camera_node:main',
            'image_resizer_node = wall_robot_pkg.image_resizer:main',
            'http_dispatcher_node = wall_robot_pkg.http_dispatcher:main',
            'server_node = wall_robot_pkg.http_dispatcher:main',
            'chassis_driver_node = wall_robot_pkg.chassis_driver_node:main',
            'vision_processor_node = wall_robot_pkg.vision_processor_node:main',
            'water_motor_test = wall_robot_pkg.water_motor_test:main',
            'mechanism_driver_node = wall_robot_pkg.mechanism_driver_node:main',
        ],
    },
)