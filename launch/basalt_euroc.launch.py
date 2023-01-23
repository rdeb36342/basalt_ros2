#  ----------------------------------------------------------------------------
#  2020 Bernd Pfrommer bernd.pfrommer@gmail.com
#
import launch
import launch.actions
import launch.substitutions
import launch_ros.actions
from os.path import expanduser

def generate_launch_description():
      
    cal_file = expanduser('~/bas_repo/src/basalt_ros/basalt_ros2/config/euroc_calib.json')
    vio_file = expanduser('~/bas_repo/src/basalt_ros/basalt_ros2/config/euroc_vio.json')    
        
    return launch.LaunchDescription([
            launch_ros.actions.Node(
                package='basalt_ros2',
                executable='basalt_vio_node',
                name='basalt_vio',
                output='screen',
                #prefix=['gdbserver localhost:3000'],
                parameters=[{
                         'calibration_file': cal_file,
                         'vio_config_file': vio_file,
                         'debug_vio': False,
                         'debug_bad_data': False,
	                     # 'extra_translation': [0.0, 0.0, 0.0],
                         # rotation is in format [w, x, y, z]
	                     # 'extra_rotation': [0.0, 0.0, 0.0, 1.0],
                }],
                remappings=[('/left_image', '/cam0/image_raw'),
                        ('/right_image', '/cam1/image_raw'),
                        ('/imu_topics', '/imu0'),
                        ('/odom', '/basalt_vio/odom')
                 ]
                # remappings=[('/left_image', '/dora_cam/left/image_raw'),
                #         ('/right_image', '/dora_cam/right/image_raw'),
                #         ('/gyro', '/dora/imu'),
                #         ('/accel', '/dora/imu'),
                #         ('/odom', '/basalt_vio/odom')
                #  ]                 
        )    
    ])