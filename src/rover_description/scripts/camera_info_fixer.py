#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from sensor_msgs.msg import CameraInfo

class CameraInfoFixer(Node):
    def __init__(self):
        super().__init__('camera_info_fixer')
        
        self.declare_parameter('baseline', 0.1)
        self.baseline = self.get_parameter('baseline').get_parameter_value().double_value
        
        sub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.VOLATILE)
        pub_qos = QoSProfile(depth=10, reliability=ReliabilityPolicy.RELIABLE, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        
        self.sub = self.create_subscription(CameraInfo, '/stereo/right/camera_info_raw', self.listener_callback, sub_qos)
        self.pub = self.create_publisher(CameraInfo, '/stereo/right/camera_info', pub_qos)
        self.get_logger().info(f'CameraInfoFixer running. Baseline: {self.baseline}m')

    def listener_callback(self, msg: CameraInfo):
        # Force the exact TF frame ID expected by ROS stereo processing
        msg.header.frame_id = 'left_camera_optical_frame'
        
        fx = msg.p[0] if msg.p[0] != 0.0 else msg.k[0]
        if fx == 0.0:
            fx = 476.701
            
        p_list = list(msg.p)
        if len(p_list) < 12:
            p_list = [0.0] * 12
            p_list[0] = fx
            p_list[5] = fx
            p_list[2] = msg.width / 2.0 if msg.width > 0 else 320.0
            p_list[6] = msg.height / 2.0 if msg.height > 0 else 240.0
            p_list[10] = 1.0

        p_list[3] = -fx * self.baseline
        msg.p = p_list
        
        self.pub.publish(msg)

def main(args=None):
    rclpy.init(args=args)
    node = CameraInfoFixer()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()