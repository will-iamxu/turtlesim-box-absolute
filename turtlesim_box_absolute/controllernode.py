import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class Controller_Node(Node):
    def __init__(self):
        super().__init__('turt_controller')
        self.get_logger().info("Node Started")

        self.move_distance = 2.0
        self.current_distance = 0.0
        self.desired_theta_change = math.pi / 2
        self.turn_threshold = 0.00000005
        self.move_threshold = 0.1

        self.initial_x = None
        self.initial_y = None
        self.initial_theta = None
        self.target_theta = None  # Target orientation in radians

        self.Kp_theta = 2.0

        self.sequence = ['move', 'turn', 'move', 'turn', 'move', 'turn', 'move', 'turn']
        self.current_step = 0

        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    def pose_callback(self, msg):
        if self.initial_x is None or self.initial_y is None or self.initial_theta is None:
            self.initial_x = msg.x
            self.initial_y = msg.y
            self.initial_theta = msg.theta
            self.target_theta = self.normalize_angle(msg.theta)  # Initialize target_theta with current orientation

        if self.sequence[self.current_step] == 'move':
            self.move_forward(msg)
        elif self.sequence[self.current_step] == 'turn':
            self.turn_to_absolute_direction(msg)

    def move_forward(self, msg):
        self.current_distance = math.sqrt((msg.x - self.initial_x) ** 2 + (msg.y - self.initial_y) ** 2)
        if self.current_distance >= self.move_distance:
            self.current_step = (self.current_step + 1) % len(self.sequence)
            self.initial_theta = msg.theta
            self.command_velocity(0.0, 0.0)
            self.initial_x = msg.x
            self.initial_y = msg.y
        else:
            self.command_velocity(1.0, 0.0)

    def turn_to_absolute_direction(self, msg):
        if self.current_step == 1:  # Assuming East
            self.target_theta = 0  # East
        elif self.current_step == 3:  # South
            self.target_theta = math.pi / 2  # South
        elif self.current_step == 5:  # West
            self.target_theta = math.pi  # West
        elif self.current_step == 7:  # North
            self.target_theta = -math.pi / 2  # North
        
        angle_turned = self.normalize_angle(msg.theta - self.initial_theta)
        desired_turn = self.normalize_angle(self.target_theta - self.initial_theta)
        turning_error = desired_turn - angle_turned

        if abs(turning_error) <= self.turn_threshold:
            self.current_step = (self.current_step + 1) % len(self.sequence)
            self.command_velocity(0.0, 0.0)
            self.initial_x = msg.x
            self.initial_y = msg.y
            if self.current_step == 0:  # Reset for next move after completing the box
                self.initial_theta = None
        else:
            self.command_velocity(0.0, self.Kp_theta * turning_error)

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
