import rclpy
from rclpy.node import Node
from poliba_interfaces.msg import UnicycleCmd, RobotStatus

class UnicycleCmdPublisher(Node):
    def __init__(self):
        super().__init__('unicycle_cmd_publisher')

        # Publisher
        self.publisher_ = self.create_publisher(UnicycleCmd, '/unicycle_cmd', 10)
        self.subscriber_ = self.create_subscription(
            RobotStatus,
            '/robot_status',
            self.status_callback,
            10
        )

        # Publish at 1 Hz
        self.timer = self.create_timer(1.0, self.publish_command)

        # Command values
        self.linear_velocity = 1.0
        self.angular_velocity = 0.5

    def status_callback(self, msg: RobotStatus):
        # if the battery level is low, stop the robot
        if msg.battery_level < 30.0:
            self.linear_velocity = 0.0
            self.angular_velocity = 0.0
            self.get_logger().warning('Battery low, stopping the robot.')
        else:
            # Resume normal operation
            self.linear_velocity = 1.0
            self.angular_velocity = 0.5
            self.get_logger().info('Battery level is sufficient, sending normal velocities.')

    def publish_command(self):
        msg = UnicycleCmd()
        msg.linear_velocity = float(self.linear_velocity)
        msg.angular_velocity = float(self.angular_velocity)

        self.publisher_.publish(msg)
        self.get_logger().info(f'Publishing UnicycleCmd: linear={msg.linear_velocity:.2f}, angular={msg.angular_velocity:.2f}')

def main(args=None):
    rclpy.init(args=args)
    node = UnicycleCmdPublisher()
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
