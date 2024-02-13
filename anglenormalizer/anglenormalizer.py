import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class Controller_Node(Node):
    def __init__(self):
        super().__init__('turt_controller')
        self.get_logger().info("Node Started")

        # Movement control flags
        self.is_moving_forward = True
        self.is_turning = False

        # Movement parameters
        self.move_distance = 2.0  
        self.current_distance = 0.0
        self.desired_theta_change = math.pi / 2  # 90 degrees turn for each corner
        self.turn_threshold = 0.01  # Threshold for completing the turn
        self.move_threshold = 0.1   # Threshold for completing the straight movement

        self.initial_x = None
        self.initial_y = None
        self.initial_theta = None

        self.Kp_theta = 2.0

        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def pose_callback(self, msg):
        if self.initial_x is None or self.initial_y is None or self.initial_theta is None:
            self.initial_x = msg.x
            self.initial_y = msg.y
            self.initial_theta = msg.theta

        if self.is_moving_forward:
            self.current_distance = math.sqrt((msg.x - self.initial_x) ** 2 + (msg.y - self.initial_y) ** 2)
            if self.current_distance >= self.move_distance:
                self.is_moving_forward = False
                self.is_turning = True
                self.initial_theta = msg.theta  
                self.command_velocity(0.0, 0.0)  
            else:
                self.command_velocity(1.0, 0.0)   
        elif self.is_turning:
            angle_turned = self.normalize_angle(msg.theta - self.initial_theta)
            if abs(angle_turned) >= self.desired_theta_change:
                self.is_turning = False
                self.is_moving_forward = True
                self.initial_x = msg.x  
                self.initial_y = msg.y
                self.command_velocity(0.0, 0.0)  
            else:
                self.command_velocity(0.0, self.Kp_theta * (self.desired_theta_change - angle_turned))

    def command_velocity(self, linear_v, angular_v):
        msg = Twist()
        msg.linear.x = linear_v
        msg.angular.z = angular_v
        self.velocity_publisher.publish(msg)

    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        while angle > math.pi:
            angle -= 2.0 * math.pi
        while angle < -math.pi:
            angle += 2.0 * math.pi
        return angle

def main(args=None):
    rclpy.init(args=args)
    node = Controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
