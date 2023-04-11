from launch import LaunchDescription
import launch_ros.actions


def generate_launch_description():
    return LaunchDescription([
        launch_ros.actions.Node(
            package ='hesai_lidar',
            namespace ='hesai',
            executable ='hesai_lidar_node',
            name ='hesai_node',
            output ="screen",
            emulate_tty=True,
            parameters=[
                # PCAP file path needs to be the relative path from the root of the hesai_lidar package (i.e. "pcapFiles/test_lidar.pcap")
                {"pcap_file": ""},
                {"server_ip"  : "192.168.1.201"},
                {"lidar_recv_port"  : 2368},
                {"gps_port"  : 10110},
                {"start_angle"  : 0.0},
                {"lidar_type"  : "PandarXT-32"},
                {"frame_id"  : "PandarXT-32"},
                {"pcldata_type"  : 0},
                {"publish_type"  : "points"},
                {"timestamp_type"  : ""},
                {"data_type"  : ""},
                {"lidar_correction_file"  : "./src/1.Drivers/hesai_lidar/config/PandarXT-32.csv"},
                {"multicast_ip"  : ""},
                {"coordinate_correction_flag"  : False},
                {"fixed_frame"  : 'lidar'},
                {"target_frame_frame"  : ""}
            ]
        )
    ])



