from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    return LaunchDescription([
        # 1. IMU e Sensores de Baixo Nível (Hardware)
        Node(
            package='robonejopy',
            executable='imu_node',
            name='imu_node',
            output='screen'
        ),

        Node(
            package='robonejopy',
            executable='cameraPub',
            name='cameraPub',
            output='screen'
        ),
        # 2. Nó de Pose (Calcula a orientação para o Foxglove)
        Node(
            package='robonejopy',
            executable='pose_publisher_node',
            name='pose_publisher_node'
        ),
        # 3. Inteligência e Navegação (Os nós que blindamos)
        Node(
            package='robonejopy',
            executable='dead_reckoning_node', # O nó com auto-calibração e PoseStamped
            name='dead_reckoning',
            output='screen'
        ),

        Node(
            package='robonejopy',
            executable='lane_detector_node', # O nó com fail-safe de 15 frames
            name='lane_detector_node',
            output='screen'
        ),

        # 4. Transformações Estáticas (TF)
        # É vital que o frame_id da Odometria coincida com o do Static TF
        # map -> odom -> base_link -> imu_link/camera_link
        
        # Conecta o mapa ao link principal do robô (necessário para o Foxglove 3D)
        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='map_to_odom',
            arguments=['0', '0', '0', '0', '0', '0', 'map', 'odom']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='odom_to_base',
            arguments=['0', '0', '0', '0', '0', '0', 'odom', 'base_link']
        ),

        Node(
            package='tf2_ros',
            executable='static_transform_publisher',
            name='base_to_camera',
            arguments=['0.1', '0', '0.15', '0', '0', '0', 'base_link', 'camera_link']
        )
    ])