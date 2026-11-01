import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Pose
from std_msgs.msg import Bool
import math

class ResetNode(Node):
    def __init__(self):
        super().__init__('reset_node')
        self.subscription = self.create_subscription(Pose, '/pose', self.pose_callback, 10)
        self.publisher = self.create_publisher(Bool, '/reset', 10)

    def pose_callback(self, msg):
        # Calcolo della distanza euclidea dall'origine
        dist = math.sqrt(msg.position.x**2 + msg.position.y**2)
        
        if dist > 6.0:
            reset_msg = Bool()
            reset_msg.data = True
            self.publisher.publish(reset_msg)
            self.get_logger().info('Distanza > 6.0m! Inviato segnale di reset.')

def main(args=None):
    rclpy.init(args=args)
    node = ResetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()