from launch import LaunchDescription
from launch_ros.actions import Node
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import AnyLaunchDescriptionSource
from launch.launch_description_sources import PythonLaunchDescriptionSource
from ament_index_python.packages import get_package_share_directory
import os

def generate_launch_description():

    # 1. 启动 Realsense 相机 (带深度对齐)
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('realsense2_camera'),
                'launch',
                'rs_launch.py'
            )
        ),
        launch_arguments={
            'align_depth.enable': 'true',
            'rgb_camera.profile': '640x480x30',
            'depth_module.profile': '640x480x30'
        }.items()
    )
    
    # 2. 启动 Rosbridge (负责 WebSocket 通信)
    rosbridge_launch = IncludeLaunchDescription(
        AnyLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory('rosbridge_server'),
                'launch',
                'rosbridge_websocket_launch.xml'
            )
        )
    )

    return LaunchDescription([
        # === 基础组件 ===
        realsense_launch,
        rosbridge_launch,

        # === 视频流服务器 ===
        Node(
            package='web_video_server',
            executable='web_video_server',
            name='web_video_server',
            parameters=[{'port': 8090}]
        ),

        # === 传感器节点 ===
        Node(
            package='wall_robot_pkg',
            executable='sick_odom_node',
            name='sick_odom_node',
            output='screen'
        ),

        # ==========================================
        # 🌟 核心控制节点 (已解耦)
        # ==========================================
        # 1. 全车通信网关
        Node(
            package='wall_robot_pkg',
            executable='server_node',
            name='server_node',
            output='screen'
        ),

        # 2. 主控大脑
        Node(
            package='wall_robot_pkg',
            executable='mission_controller_node',
            name='mission_controller_node',
            output='screen'
        ),

        # 3. 底盘驱动
        Node(
            package='wall_robot_pkg',
            executable='chassis_driver_node',
            name='chassis_driver_node',
            output='screen'
        ),
        
        # 4. 独立底层机构驱动 (M1/M2/IG35/推杆)
        Node(
            package='wall_robot_pkg',
            executable='mechanism_driver_node',
            name='mechanism_driver_node',
            output='screen'
        ),
        
        # 5. 视觉处理
        Node(
            package='wall_robot_pkg', 
            executable='vision_processor_node',
            name='vision_processor_node',
            parameters=[{
                'fx': 607.035, 'fy': 607.062, 'cx': 322.111, 'cy': 240.756,
                'cam_to_lidar_dx': 0.2  
            }],
            output='screen'
        ),

        # 图像压缩/转换 (如果需要)
        Node(
            package='wall_robot_pkg',
            executable='image_resizer_node',
            name='image_resizer_node',
            output='screen'
        )
    ])