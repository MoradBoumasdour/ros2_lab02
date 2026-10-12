import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist, Pose

class LocalizationNode(Node):
    def __init__(self):
        super().__init__('localization')
        
        # Sottoscrittore al topic delle velocità
        self.subscription = self.create_subscription(
            Twist,
            '/cmd_vel',
            self.cmd_vel_callback,
            10)
            
        # Pubblicatore per la posizione stimata
        self.publisher_ = self.create_publisher(Pose, '/pose', 10)
        
        # Inizializzazione della posizione all'origine degli assi
        self.x = 0.0
        self.y = 0.0
        
        # Intervallo di tempo (1 secondo) tra un comando e l'altro
        self.dt = 1.0

    def cmd_vel_callback(self, msg):
        # Stima della nuova posizione
        self.x += msg.linear.x * self.dt
        self.y += msg.linear.y * self.dt
        
        # Creazione del messaggio Pose
        pose_msg = Pose()
        pose_msg.position.x = self.x
        pose_msg.position.y = self.y
        pose_msg.position.z = 0.0
        
        # I quaternioni dell'orientamento rimangono a 0 (tranne w=1.0 per una posa valida, anche se opzionale qui)
        pose_msg.orientation.w = 1.0
        
        # Pubblicazione e logging
        self.publisher_.publish(pose_msg)
        self.get_logger().info(f'Posa stimata pubblicata: X={self.x}, Y={self.y}')

def main(args=None):
    rclpy.init(args=args)
    node = LocalizationNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()