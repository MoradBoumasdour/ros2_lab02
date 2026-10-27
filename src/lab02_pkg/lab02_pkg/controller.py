import rclpy
import numpy as np
import math
import tf_transformations
from rclpy.node import Node
from  sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from rclpy.qos import qos_profile_sensor_data
from math import ceil


class ControllerNode(Node):

    def __init__(self):
        super().__init__('controller')

        self.velMax = 0.22
        self.velAngMax = 1.5
        self.loopControl = 0.2
        self.bumperThreshold = 0.7
        self.state = 'FORWARD'
        self.current_yaw = 0.0
        self.target_yaw = 0.0
        self.exact_yaw = 0.0
        #publisher of robot's movement
        self.publisher = self.create_publisher(Twist, 'cmd_vel', 10)
        #read Laidar tiopic
        self.create_subscription(LaserScan, "scan", self.LidarCallback,
        qos_profile_sensor_data)

        #read Odometry topic
        self.odom_sub = self.create_subscription(
            Odometry,
            'odom',
            self.odomCallback,
            10)
        #read real odometry of robot
        # self.ground_truth_sub = self.create_subscription(
        #     Odometry,
        #     'ground_truth',
        #     self.groundTruthCallback,
        #     10)

    # The LidrCallback function reads the ranges from LaserScan message of the Lidar,
    # caps the values to avoid problems with inf and nan values.
    # It then calls the moving function to control the robot's movement based on the processed ranges.
    def LidarCallback(self, msg):
        #self.get_logger().info("Recieved ranges from Lidar")
        capmax = 3.5
        capmin = 0.12
        ranges_np = np.array(msg.ranges) #transform in numPy array
        angle_min = msg.angle_min
        angle_max = msg.angle_max
        angle_increment = msg.angle_increment

        badRangesMax = np.isinf(ranges_np) | np.isnan(ranges_np) #capping the values of the ranges to avoid problems with inf and nan values
        badRangesMin = ranges_np < capmin 
        ranges_np[badRangesMax] = capmax
        ranges_np[badRangesMin] = capmin

        self.moving(ranges_np, angle_min, angle_max, angle_increment)

    # The odomCallback function reads the orientation from the Odometry message,
    # converts the quaternion to Euler angles, and updates the current yaw of the robot.  
    def odomCallback(self, msg):
        quat = [
            msg.pose.pose.orientation.x, 
            msg.pose.pose.orientation.y, 
            msg.pose.pose.orientation.z, 
            msg.pose.pose.orientation.w
        ]
        _, _, self.current_yaw = tf_transformations.euler_from_quaternion(quat)

    # The groundTruthCallback function reads the orientation from the ground truth Odometry message,
    # converts the quaternion to Euler angles, and updates the exact yaw of the robot.
    # It also calculates the yaw error between the current yaw and the exact yaw, normalizes it to avoid jumps at 360 degrees, and logs the error.
    # def groundTruthCallback(self, msg):
    #     quat = [
    #         msg.pose.pose.orientation.x, 
    #         msg.pose.pose.orientation.y, 
    #         msg.pose.pose.orientation.z, 
    #         msg.pose.pose.orientation.w
    #     ]
    #     _, _, self.exact_yaw = tf_transformations.euler_from_quaternion(quat)
        
        
    #     yaw_error = self.current_yaw - self.exact_yaw
        
    #     # Normalizzazione dell'errore per evitare salti a 360 gradi
    #     if yaw_error > math.pi:
    #         yaw_error -= 2 * math.pi
    #     elif yaw_error < -math.pi:
    #         yaw_error += 2 * math.pi
            
    #     self.get_logger().info(f"Error (Yaw): {yaw_error:.4f} rad")

    # The moving function controls the robot's movement based on the current state and the processed ranges from the Lidar.
    # In the 'FORWARD' state, it checks the distance to obstacles in front of the robot and decides whether to move forward or switch to the 'TURNING' state.
    # In the 'TURNING' state, it calculates the error between the target yaw and the current yaw, and adjusts the robot's angular velocity to rotate towards the target yaw.  
    def moving(self, ranges, angle_min, angle_max, angle_increment):
        msg = Twist()
        rangeTotal = 0.174533 #10 degrees (in rad) of range for forward and side scan
        indexRange = math.ceil(rangeTotal/(2*angle_increment)) #calculates the half of index in order to get  total 10 degrees range
        if (angle_min<0.0872665 and angle_max>(6.28319-0.0872665) ): #if angle min is smaller then 5 degrees an angle max greater than 355 degrees the data are used other way discarded
            if self.state == 'FORWARD':
                front_scan = np.concatenate((ranges[0:indexRange], ranges[-indexRange:]))
                front_dist = np.median(front_scan) #using np.median to avoid outliers in the front scan given by gaussian errors in the lidar readings                     
                if front_dist > self.bumperThreshold:
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
                    left_idx =math.ceil(num_ranges * 0.25)
                    right_idx = math.ceil(num_ranges * 0.75)

                    dist_left = np.median(ranges[left_idx-indexRange : left_idx+indexRange])#using np.median to avoid outliers in the front scan given by gaussian errors in the lidar readings              
                    dist_right = np.median(ranges[right_idx-indexRange : right_idx+indexRange])#using np.median to avoid outliers in the front scan given by gaussian errors in the lidar readings              
                    
                    #for debug
                    self.get_logger().info(f"distance to the right:{dist_right:.2f}")
                    self.get_logger().info(f"distance to the left:{dist_left:.2f}")
                    
                    if dist_left>dist_right:
                        self.target_yaw = self.current_yaw + (math.pi / 2.0)
                        self.turn_direction = 1.0 # counterclockwise rotation
                    else:
                        self.target_yaw = self.current_yaw - (math.pi / 2.0)
                        self.turn_direction = -1.0 # clockwise rotation
                    #normalization around pi -pi
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
                
# The main function initializes the ROS2 node, creates an instance of the ControllerNode, and starts spinning to process incoming messages.
# It also ensures proper cleanup by destroying the node and shutting down ROS2 when the program is terminated.
def main(args = None):
    rclpy.init(args=args)
    node = ControllerNode()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()