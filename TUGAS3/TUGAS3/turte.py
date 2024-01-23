import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from turtlesim.srv import TeleportAbsolute
from std_srvs.srv import Empty
import math

traverse = [0.00, 4.00, 0.00, 4.00, 0.00, 4.00, 0.00, 6.18, 0.00, 0.00, 6.18, 0.00, 6.18]
rotate = [2.08, 0.00, 2.08, 0.00, 2.08, 0.00, 0.52, 3.09, 0.00, -1.04, 3.09, -1.04, 3.09]

class PublisherNode(Node):

    def __init__(self):
        super().__init__('publisher')
        self.publisher = self.create_publisher(Twist, 'turtle1/cmd_vel', 10)
        self.teleport_absolute_client = self.create_client(TeleportAbsolute, '/turtle1/teleport_absolute')
        self.clear_turtlesim_client = self.create_client(Empty, '/clear')
        timer_period = 2  # seconds
        self.timer = self.create_timer(timer_period, self.timer_callback)
        self.i = 0

        # Wait for services to be available
        self.wait_for_services()

        # Initial movement to reach starting coordinates
        self.teleport_absolute(7.5, 5.0, 0.0)
        self.clear_turtlesim()

    def wait_for_services(self):
        while not self.teleport_absolute_client.wait_for_service(timeout_sec=1.0) or \
                not self.clear_turtlesim_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('Waiting for services...')

    def teleport_absolute(self, x, y, theta):
        request = TeleportAbsolute.Request()
        request.x = x
        request.y = y
        request.theta = theta

        future = self.teleport_absolute_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('Turtle teleported to x={}, y={}, theta={}'.format(x, y, theta))
        else:
            self.get_logger().error('Teleport service call failed')

    def clear_turtlesim(self):
        request = Empty.Request()

        future = self.clear_turtlesim_client.call_async(request)
        rclpy.spin_until_future_complete(self, future)

        if future.result() is not None:
            self.get_logger().info('TurtleSim cleared')
        else:
            self.get_logger().error('Clear TurtleSim service call failed')

    def timer_callback(self):
        if self.i < len(traverse):
            msg = Twist()
            msg.linear.x = traverse[self.i]
            msg.angular.z = rotate[self.i]
            self.publisher.publish(msg)
            self.i += 1
        else:
            self.get_logger().info('Finished the movement sequence.')

def main(args=None):
    rclpy.init(args=args)

    minimal_publisher = PublisherNode()

    rclpy.spin(minimal_publisher)

    minimal_publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
