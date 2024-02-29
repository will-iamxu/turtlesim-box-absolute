import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
import math

class Controller_Node(Node):
    def __init__(self):
        super().__init__('turt_controller')
        self.get_logger().info("Controller Node Started")
        
        # Movement parameters
        self.move_distance = 2.0
        self.current_distance = 0.0
        self.desired_theta_change = math.pi / 2
        self.turn_threshold = 0.0000001
        self.move_threshold = 0.1
        
        # Initial pose
        self.initial_x = None
        self.initial_y = None
        self.initial_theta = None
        self.target_theta = None
        
        # Control parameters
        self.Kp_theta = 2.0
        self.Kp_distance = 0.3
        
        # Direction management
        self.direction_sequence = ['north', 'east', 'south', 'west', 'west', 'north', 'east', 'south', 'south', 'east','north','west','west','south','east','north']
        self.current_direction_index = 0
        self.moving_straight = False
        
        # ROS subscribers and publishers
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.get_logger().info("Subscribers and Publishers Created")

    def pose_callback(self, msg):
        self.get_logger().info(f"Pose callback triggered: {msg}")
        
        if self.initial_x is None or self.initial_y is None or self.initial_theta is None:
            self.initial_x = msg.x
            self.initial_y = msg.y
            self.initial_theta = msg.theta
            # Set initial target_theta based on the first direction in the sequence
            directions = {'east': 0, 'north': math.pi / 2, 'west': math.pi, 'south': -math.pi / 2}
            self.target_theta = directions[self.direction_sequence[self.current_direction_index]]
            self.get_logger().info(f"Initial pose set to x: {self.initial_x}, y: {self.initial_y}, theta: {self.initial_theta}")
            self.face_direction(msg, self.direction_sequence[self.current_direction_index])  # Ensure facing the correct initial direction

        if self.moving_straight:
            self.move_straight(msg)
        else:
            self.face_direction(msg, self.direction_sequence[self.current_direction_index])

    def move_straight(self, msg):
        # Calculate the distance moved so far
        distance_moved = math.sqrt((msg.x - self.initial_x) ** 2 + (msg.y - self.initial_y) ** 2)
        self.current_distance = distance_moved
        
        # Calculate the remaining distance
        remaining_distance = self.move_distance - self.current_distance
        
        # Check if the turtle is within a small threshold of the target position
        if remaining_distance > self.move_threshold:
            # Calculate the proportional velocity
            linear_v = max(self.Kp_distance * remaining_distance, 0.1)
            self.command_velocity(linear_v, 0.0)  # Move forward with the proportional velocity
        else:
            # Stop moving as the turtle is close enough to the target position
            self.command_velocity(0.0, 0.0)
            self.moving_straight = False  # Ready to start turning
            # Reset distance and initial position for the next movement
            self.current_distance = 0.0
            self.initial_x = msg.x
            self.initial_y = msg.y
            self.initial_theta = msg.theta

    def normalize_angle(self, angle):
        """Normalize an angle to [-pi, pi]."""
        return math.atan2(math.sin(angle), math.cos(angle))

    def face_direction(self, msg, direction):
        directions = {'east': 0, 'north': math.pi / 2, 'west': math.pi, 'south': -math.pi / 2}
        self.target_theta = directions[direction]

        current_orientation = self.normalize_angle(msg.theta)
        desired_orientation = self.normalize_angle(self.target_theta)
        orientation_error = self.normalize_angle(desired_orientation - current_orientation)

        self.get_logger().info(f"Facing {direction}, current orientation: {current_orientation}, desired orientation: {desired_orientation}, error: {orientation_error}")

        if abs(orientation_error) <= self.turn_threshold:
            self.get_logger().info(f"Correctly facing {direction} within the threshold.")
            self.moving_straight = True  # Start moving straight after turning
            self.current_direction_index = (self.current_direction_index + 1) % len(self.direction_sequence)
        else:
            # Ensure the angular velocity command is significant enough to cause movement
            angular_v = max(min(self.Kp_theta * orientation_error, 1.0), -1.0)
            self.command_velocity(0.0, angular_v)

    def command_velocity(self, linear_v, angular_v):
        msg = Twist()
        msg.linear.x = linear_v
        msg.angular.z = angular_v
        self.velocity_publisher.publish(msg)
        self.get_logger().info(f"Commanded velocity: linear {linear_v}, angular {angular_v}")

def main(args=None):
    rclpy.init(args=args)
    node = Controller_Node()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
