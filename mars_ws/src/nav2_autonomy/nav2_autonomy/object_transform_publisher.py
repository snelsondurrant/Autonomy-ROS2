import rclpy
from rclpy.node import Node
from vision_msgs.msg import Detection3DArray, ObjectHypothesisWithPose
from geometry_msgs.msg import TransformStamped
from tf2_ros import TransformBroadcaster

class ObjectTFPublisher(Node):
    def __init__(self):
        super().__init__('object_tf_publisher')
        self.subscription = self.create_subscription(
            Detection3DArray,
            '/zed/detections',  # Adjust this topic name as needed
            self.detection_callback,
            10)

        self.last_msg = None
        self.timer = self.create_timer(0.01, self.timer_callback)
        self.tf_broadcaster = TransformBroadcaster(self)
        self.confidence_threshold = 0.7  # Set your confidence threshold here
        self.alpha = 0.1  # Smoothing factor for the low-pass filter
        self.smoothed_position = None

    
    def detection_callback(self, msg):
        self.last_msg = msg

    def timer_callback(self):
        if self.last_msg is not None:
            for detection in self.last_msg.detections:
                for hypothesis in detection.results:
                    if hypothesis.hypothesis.score >= self.confidence_threshold:
                        self.publish_object_tf(self.last_msg.header, detection, hypothesis.hypothesis)

    def publish_object_tf(self, header, detection, hypothesis):
        current_position = detection.bbox.center.position

        if self.smoothed_position is None:
            self.smoothed_position = current_position
        else:
            self.smoothed_position.x = self.alpha * current_position.x + (1 - self.alpha) * self.smoothed_position.x
            self.smoothed_position.y = self.alpha * current_position.y + (1 - self.alpha) * self.smoothed_position.y
            self.smoothed_position.z = self.alpha * current_position.z + (1 - self.alpha) * self.smoothed_position.z
        
        t = TransformStamped()
        t.header = header
        t.child_frame_id = f"{hypothesis.class_id}"
        t.transform.translation.x = detection.bbox.center.position.x
        t.transform.translation.y = detection.bbox.center.position.y
        t.transform.translation.z = detection.bbox.center.position.z
        t.transform.rotation = detection.bbox.center.orientation
        self.tf_broadcaster.sendTransform(t)

def main(args=None):
    rclpy.init(args=args)
    node = ObjectTFPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
