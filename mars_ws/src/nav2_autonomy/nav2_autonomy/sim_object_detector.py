import rclpy
from rclpy.node import Node
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Pose
from vision_msgs.msg import Detection3DArray, Detection3D, ObjectHypothesisWithPose
from math import sqrt
from rosgraph_msgs.msg import Clock
from rclpy.qos import QoSProfile, QoSReliabilityPolicy, QoSHistoryPolicy


class SimulatedDetector(Node):
    def __init__(self):
        super().__init__('simulated_detector')
        
        # Declare and set the use_sim_time parameter
        # self.declare_parameter('use_sim_time', True)

        #TODO implement sim time param
        clock_qos = QoSProfile(
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
            history=QoSHistoryPolicy.KEEP_LAST,
            depth=1
        )

        
        # Subscribe to the /clock topic
        self.create_subscription(Clock, '/clock', self.clock_callback, clock_qos)
        
        self.sim_time = self.get_clock().now()
        
        # Rest of your initialization code
        self.object_pose = Pose()
        self.object_pose.position.x = 1.0
        self.object_pose.position.y = 1.0
        self.object_pose.position.z = 0.0

        self.robot_pose_sub = self.create_subscription(
            Odometry,
            '/odometry/global',
            self.robot_pose_callback,
            10)
        
        self.detection_pub = self.create_publisher(
            Detection3DArray,
            '/zed/detections',
            10)
        
        self.detection_range = 2.5

    def clock_callback(self, msg):
        self.sim_time = rclpy.time.Time.from_msg(msg.clock)

    def robot_pose_callback(self, msg):
        distance = self.calculate_distance(msg, self.object_pose)
        if distance <= self.detection_range:
            self.publish_detection()

    def calculate_distance(self, odom1, pose2):
        dx = odom1.pose.pose.position.x - pose2.position.x
        dy = odom1.pose.pose.position.y - pose2.position.y
        dz = odom1.pose.pose.position.z - pose2.position.z
        return sqrt(dx*dx + dy*dy + dz*dz)

    def publish_detection(self):
        detection_msg = Detection3DArray()
        # implement sim time param
        detection_msg.header.stamp = self.sim_time.to_msg()
        detection_msg.header.frame_id = "camera_link"  
        detection = Detection3D()
        detection.results.append(ObjectHypothesisWithPose())
        detection.results[0].hypothesis.class_id = 'simulated_object'
        detection.results[0].hypothesis.score = 1.0
        detection.bbox.center.position = self.object_pose.position

        detection_msg.detections.append(detection)
        self.detection_pub.publish(detection_msg)

def main(args=None):
    rclpy.init(args=args)
    node = SimulatedDetector()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
