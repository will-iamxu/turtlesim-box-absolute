import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.msg import Pose
from turtlesim.srv import Kill
import math

class Controller_Node(Node):
    def __init__(self):
        super().__init__('turt_controller')
        self.get_logger().info("Controller Node Started")
        
        self.move_distance = 2.0
        self.current_distance = 0.0
        self.turn_threshold = 0.0000001
        self.move_threshold = 0.1
        
        self.initial_x = None
        self.initial_y = None
        self.initial_theta = None
        
        self.Kp_theta = 2.0
        self.Kp_distance = 0.3
        
        self.direction_sequence = ['north', 'east', 'south', 'west', 'west', 'north', 'east', 'south', 'south', 'east','north','west','west','south','east','north']
        self.current_direction_index = 0
        self.moving_straight = False
        self.pattern_completed = False
        
        self.pose_subscriber = self.create_subscription(Pose, "/turtle1/pose", self.pose_callback, 10)
        self.velocity_publisher = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)
        self.kill_client = self.create_client(Kill, '/kill')
        while not self.kill_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        self.get_logger().info("Subscribers and Publishers Created")

    def pose_callback(self, msg):
        if self.pattern_completed:
            return
        
        if self.initial_x is None:
            self.initial_x = msg.x
            self.initial_y = msg.y
            self.initial_theta = msg.theta
            self.get_logger().info(f"Initial pose set to x: {self.initial_x}, y: {self.initial_y}, theta: {self.initial_theta}")

        if self.moving_straight:
            self.move_straight(msg)
        else:
            self.face_direction(msg)

    def move_straight(self, msg):
        distance_moved = math.sqrt((msg.x - self.initial_x) ** 2 + (msg.y - self.initial_y) ** 2)
        remaining_distance = self.move_distance - distance_moved
        
        if remaining_distance > self.move_threshold:
            linear_v = max(self.Kp_distance * remaining_distance, 0.05)
            self.command_velocity(linear_v, 0.0)
        else:
            self.command_velocity(0.0, 0.0)
            self.moving_straight = False
            self.initial_x = msg.x
            self.initial_y = msg.y
            self.initial_theta = msg.theta
            self.current_direction_index += 1
            self.check_pattern_completion()

    def face_direction(self, msg):
        directions = {'east': 0, 'north': math.pi / 2, 'west': math.pi, 'south': -math.pi / 2}
        target_theta = directions[self.direction_sequence[self.current_direction_index]]
        current_orientation = self.normalize_angle(msg.theta)
        desired_orientation = self.normalize_angle(target_theta)
        orientation_error = self.normalize_angle(desired_orientation - current_orientation)
        
        self.get_logger().info(f"Facing {self.direction_sequence[self.current_direction_index]}, current orientation: {current_orientation}, desired orientation: {desired_orientation}, error: {orientation_error}")

        if abs(orientation_error) > self.turn_threshold:
            angular_v = max(min(self.Kp_theta * orientation_error, 1.0), -1.0)
            self.command_velocity(0.0, angular_v)
        else:
            self.moving_straight = True

    def check_pattern_completion(self):
        if self.current_direction_index >= len(self.direction_sequence):
            self.pattern_completed = True
            self.kill_turtle()
            self.get_logger().info("Pattern completed, initiating kill sequence.")

    def kill_turtle(self):
        if not self.kill_client.service_is_ready():
            self.get_logger().info("Kill service is not ready. Waiting...")
            return
        req = Kill.Request()
        req.name = 'turtle1'
        self.kill_client.call_async(req).add_done_callback(self.kill_turtle_callback)

    def kill_turtle_callback(self, future):
        try:
            response = future.result()
            self.get_logger().info("Turtle killed successfully.")
        except Exception as e:
            self.get_logger().error(f"Service call failed: {e}")

    def normalize_angle(self, angle):
        return math.atan2(math.sin(angle), math.cos(angle))

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
