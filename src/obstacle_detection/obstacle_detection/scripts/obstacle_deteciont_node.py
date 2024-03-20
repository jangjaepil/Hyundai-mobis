import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy
from sensor_msgs.msg import LaserScan
from geometry_msgs.msg import PoseArray, Pose, Point
from visualization_msgs.msg import Marker
import numpy as np

from dr_spaam.detector import Detector

def read_param(node, param_name, default_value):
    """
    @brief      Convenience function to read a parameter with a default value.
    """
    value = node.declare_parameter(param_name, default_value).value
    return value

class ObstacleDetectorNode(Node):
    def __init__(self):
        super().__init__('obstacle_detector_node')
        print("===============Read Parameters===============")
        self._read_params()
        print("===============Initialize Detector===============")
        self._detector = Detector(
            ckpt_file = self.weight_file,
            model = self.detector_model,
            gpu = False,
            stride = self.stride,
            panoramic_scan = self.panoramic_scan,
        )
        self._init()

    def _read_params(self):
        """
        @brief      Reads parameters from ROS 2 parameters.
        """
        self.weight_file = read_param(self, "~weight_file", "/home/mobis/mobis_ws/src/obstacle_detection/obstacle_detection/ckpts/ckpt_jrdb_ann_drow3_e40.pth")
        self.conf_thresh = read_param(self, "~conf_thresh", 0.9)
        self.stride = read_param(self, "~stride", 1)
        self.detector_model = read_param(self, "~detector_model", "DROW3")
        self.panoramic_scan = read_param(self, "~panoramic_scan", False)

    def _init(self):
        """
        @brief      Initialize ROS 2 connections.
        """
        # Publisher
        topic = read_param(self, "~publisher/detections/topic", 'detections')
        queue_size = read_param(self, "~publisher/detections/queue_size", 1000)
        self._dets_pub = self.create_publisher(PoseArray, topic, queue_size)

        topic = read_param(self, "~publisher/rviz/topic", 'rviz')
        queue_size = read_param(self, "~publisher/rviz/queue_size", 1000)
        self._rviz_pub = self.create_publisher(Marker, topic, queue_size)

        # Subscriber
        topic = read_param(self, "~subscriber/scan/topic", 'merged_scan')
        queue_size = read_param(self, "~subscriber/scan/queue_size", 1000)

        qos = QoSProfile(depth=10)
        qos.reliability = QoSReliabilityPolicy.BEST_EFFORT

        self._scan_sub = self.create_subscription(LaserScan, topic, self._scan_callback, qos)

    def _scan_callback(self, msg):
        if (
            self._dets_pub.get_subscription_count() == 0
            and self._rviz_pub.get_subscription_count() == 0
        ):
            return

        if not self._detector.is_ready():
            self._detector.set_laser_fov(
                np.rad2deg(msg.angle_increment * len(msg.ranges))
            )

        scan = np.array(msg.ranges)
        scan[scan == 0.0] = 29.99
        scan[np.isinf(scan)] = 29.99
        scan[np.isnan(scan)] = 29.99

        dets_xy, dets_cls, _ = self._detector(scan)

        conf_mask = (dets_cls >= self.conf_thresh).reshape(-1)
        dets_xy = dets_xy[conf_mask]
        dets_cls = dets_cls[conf_mask]

        dets_msg = self._detections_to_pose_array(dets_xy, dets_cls)
        dets_msg.header = msg.header
        self._dets_pub.publish(dets_msg)

        rviz_msg = self._detections_to_rviz_marker(dets_xy, dets_cls)
        rviz_msg.header = msg.header
        self._rviz_pub.publish(rviz_msg)

    def _detections_to_pose_array(self, dets_xy, dets_cls):
        pose_array = PoseArray()
        for d_xy, d_cls in zip(dets_xy, dets_cls):
            # Detector uses following frame convention:
            # x forward, y rightward, z downward, phi is angle w.r.t. x-axis
            p = Pose()
            print("d_xy : ", d_xy)
            print("d_cls : ", d_cls)
            p.position.x = float(d_xy[0])
            p.position.y = float(d_xy[1])
            p.position.z = 0.0
            pose_array.poses.append(p)

        return pose_array

    
    def _detections_to_rviz_marker(self, dets_xy, dets_cls):
        """
        @brief     Convert detection to RViz marker msg. Each detection is marked as
                a circle approximated by line segments.
        """
        msg = Marker()
        msg.action = Marker.ADD
        msg.ns = "obstacle_dection"
        msg.id = 0
        msg.type = Marker.LINE_LIST

        # set quaternion so that RViz does not give warning
        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        msg.scale.x = 0.03  # line width
        # red color
        msg.color.r = 1.0
        msg.color.a = 1.0

        # circle
        r = 0.4
        ang = np.linspace(0, 2 * np.pi, 20)
        xy_offsets = r * np.stack((np.cos(ang), np.sin(ang)), axis=1)

        # to msg
        for d_xy, d_cls in zip(dets_xy, dets_cls):
            for i in range(len(xy_offsets) - 1):
                # start point of a segment
                p0 = Point()
                p0.x = d_xy[0] + xy_offsets[i, 0]
                p0.y = d_xy[1] + xy_offsets[i, 1]
                p0.z = 0.0
                msg.points.append(p0)

                # end point
                p1 = Point()
                p1.x = d_xy[0] + xy_offsets[i + 1, 0]
                p1.y = d_xy[1] + xy_offsets[i + 1, 1]
                p1.z = 0.0
                msg.points.append(p1)

        return msg

def main(args=None):
    rclpy.init(args=args)
    obstacle_detector_node = ObstacleDetectorNode()
    rclpy.spin(obstacle_detector_node)
    obstacle_detector_node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
