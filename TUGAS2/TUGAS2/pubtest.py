import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class AlternatingPublishers(Node):
    def __init__(self):
        super().__init__('alternating_publishers')
        self.publisher1 = self.create_publisher(Bool, 'publisher_1', 10)
        self.timer1 = self.create_timer(2, self.publish_bool1)
        self.bool_value1 = True
        self.timer_count1 = 0

        self.publisher2 = self.create_publisher(Bool, 'publisher_2', 10)
        self.timer2 = self.create_timer(3, self.publish_bool2)
        self.bool_value2 = True
        self.timer_count2 = 0

    def publish_bool1(self):
        msg1 = Bool()
        msg1.data = self.bool_value1
        self.publisher1.publish(msg1)
        self.get_logger().info('Publisher - 1 - (%d sec) -> "%s"' % (self.timer_count1, msg1.data))
        self.bool_value1 = not self.bool_value1
        self.timer_count1 += 1

    def publish_bool2(self):
        msg2 = Bool()
        msg2.data = self.bool_value2
        self.publisher2.publish(msg2)
        self.get_logger().info('Publisher - 2 - (%d sec) -> "%s"' % (self.timer_count2, msg2.data))
        self.bool_value2 = not self.bool_value2
        self.timer_count2 += 1


def main(args=None):
    rclpy.init(args=args)
    alternating_publishers = AlternatingPublishers()
    rclpy.spin(alternating_publishers)
    alternating_publishers.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
