import rclpy
import numpy as np
import math
import tf_transformations
from rclpy.node import Node
from  sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist

class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller')

        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        self.velMax = 0.22
        self.velAngMax = 1.5
        self.loopControl = 0.2
        self.bumperThreshold = 0.7
        self.state = 'FORWARD'
        self.current_yaw = 0.0
        self.target_yaw = 0.0

        self.subscription = self.create_subscription(
            LaserScan,
            'scan',
            self.LidarCallback,
            10)
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odomCallback,
            10)

    def LidarCallback(self, msg):
        self.get_logger().info("Recieved from Lidar:")
        capmax = 3.5
        capmin = 0.12
        self.get_logger().info(f"dimesnione:{len(msg.ranges)}")
        cappedRanges = [min(valore, capmax) for valore in msg.ranges]
        cappedRanges = [max(valore, capmin) for valore in cappedRanges]
        self.moving(cappedRanges)
        
    def odomCallback(self, msg):
        quat = [
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w
        ]
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(quat)

    def moving(self, ranges):
        
        msg = Twist()
        if self.state == 'FORWARD':
            if min(ranges[0:5] + ranges[-5:]) > self.bumperThreshold:
                msg.linear.x = self.velMax
                msg.angular.z = 0.0
                self.publisher.publish(msg)
                self.get_logger().info("Moving forward")
            else:
                self.get_logger().info("Obstacle detected")
                self.state = 'TURNING'
                msg.linear.x = 0.0
                self.publisher.publish(msg)
                num_ranges = len(ranges)
                left_idx = int(num_ranges * 0.25)
                right_idx = int(num_ranges * 0.75)
                dist_left = min(ranges[left_idx-5 : left_idx+5])
                dist_right = min(ranges[right_idx-5 : right_idx+5])
                
                #da levare per debug
                self.get_logger().info(f"distanza a destra:{dist_right}")
                self.get_logger().info(f"distanza a sinistra:{dist_left}")
                
                if dist_left>dist_right:
                    self.target_yaw = self.current_yaw + (math.pi / 2.0)
                    self.turn_direction = 1.0 # counterclockwise rotation
                else:
                    self.target_yaw = self.current_yaw - (math.pi / 2.0)
                    self.turn_direction = -1.0 # clockwise rotation
                #normalization around 180 -180
                if self.target_yaw > math.pi:
                    self.target_yaw -= 2 * math.pi
                elif self.target_yaw < -math.pi:
                    self.target_yaw += 2 * math.pi

                

        elif self.state == 'TURNING':
            error = self.target_yaw - self.current_yaw 
            if error > math.pi:
                error -= 2 * math.pi
            elif error < -math.pi:
               error += 2 * math.pi  

            if abs(error) < 0.03:
                self.get_logger().info("Rotation Finished")
                self.state = 'FORWARD'
                msg.linear.x = 0.0
                msg.angular.z = 0.0
            else:
                msg.linear.x = 0.0
                angRel = abs(error)/0.2
                if angRel < self.velAngMax:
                    msg.angular.z = angRel * 0.5 * self.turn_direction
                else:
                    msg.angular.z = self.velAngMax * 0.5 * self.turn_direction
                
            self.publisher.publish(msg)    
                

def main(args = None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()