import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from std_msgs.msg import Bool

class ControllerResetNode(Node):
    def __init__(self):
        super().__init__('controller_reset')
        self.publisher_ = self.create_publisher(Twist, '/cmd_vel', 10)
        self.sub_reset = self.create_subscription(Bool, '/reset', self.reset_callback, 10)
        self.timer = self.create_timer(1.0, self.timer_callback)
        
        self.N = 1
        self.current_state = 0
        self.ticks_in_current_state = 0
        
    def reset_callback(self, msg):
        if msg.data:
            self.N = 1
            self.current_state = 0
            self.ticks_in_current_state = 0
            self.get_logger().info('Ricevuto reset: riparto da N=1 e stato iniziale.')

    def timer_callback(self):
        msg = Twist()
        if self.current_state == 0:
            msg.linear.x, msg.linear.y = 1.0, 0.0
        elif self.current_state == 1:
            msg.linear.x, msg.linear.y = 0.0, 1.0
        elif self.current_state == 2:
            msg.linear.x, msg.linear.y = -1.0, 0.0
        elif self.current_state == 3:
            msg.linear.x, msg.linear.y = 0.0, -1.0
            
        self.publisher_.publish(msg)
        
        self.ticks_in_current_state += 1
        if self.ticks_in_current_state >= self.N:
            self.ticks_in_current_state = 0
            self.current_state += 1
            if self.current_state > 3:
                self.current_state = 0
                self.N += 1

def main(args=None):
    rclpy.init(args=args)
    node = ControllerResetNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()