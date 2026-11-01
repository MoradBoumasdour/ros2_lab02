import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose
from std_msgs.msg import Bool

class LocalizationResetNode(Node):
    def __init__(self):
        super().__init__('localization_reset')
        self.sub_vel = self.create_subscription(Twist, '/cmd_vel', self.cmd_vel_callback, 10)
        self.sub_reset = self.create_subscription(Bool, '/reset', self.reset_callback, 10)
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)
        
        self.x = 0.0
        self.y = 0.0
        self.dt = 1.0

    def reset_callback(self, msg):
        if msg.data:
            self.x = 0.0
            self.y = 0.0
            self.get_logger().info('Ricevuto reset: posizione azzerata a (0,0).')

    def cmd_vel_callback(self, msg):
        self.x += msg.linear.x * self.dt
        self.y += msg.linear.y * self.dt
        
        pose_msg = Pose()
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = 0.0
        pose_msg.orientation.w = 1.0
        
        self.publisher_.publish(pose_msg)

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationResetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()