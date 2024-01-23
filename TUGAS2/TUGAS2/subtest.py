import rclpy
from rclpy.node import Node
from std_msgs.msg import Bool

class AlternatingPublishers(Node):
    def __init__(self):
        super().__init__('alternating_publishers')
        self.publisher1 = self.create_publisher(Bool, 'topic1', 10)
        self.timer1 = self.create_timer(2, self.publish_bool1)
        self.bool_value1 = True
        self.timer_count1 = 0

        self.publisher2 = self.create_publisher(Bool, 'topic2', 10)
        self.timer2 = self.create_timer(3, self.publish_bool2)
        self.bool_value2 = True
        self.timer_count2 = 0

        self.subscription1 = self.create_subscription(
            Bool,
            'topic1',
            self.callback1,
            10
        )

        self.subscription2 = self.create_subscription(
            Bool,
            'topic2',
            self.callback2,
            10
        )

    def publish_bool1(self):
        msg = Bool()
        msg.data = self.bool_value1
        self.publisher1.publish(msg)
        self.get_logger().info(f'Published to topic1: {self.bool_value1}')
        self.bool_value1 = not self.bool_value1
        self.timer_count1 += 1

    def publish_bool2(self):
        msg = Bool()
        msg.data = self.bool_value2
        self.publisher2.publish(msg)
        self.get_logger().info(f'Published to topic2: {self.bool_value2}')
        self.bool_value2 = not self.bool_value2
        self.timer_count2 += 1

    def callback1(self, msg):
        self.process_and_result(msg.data, 'pub1')

    def callback2(self, msg):
        self.process_and_result(msg.data, 'pub2')

    def process_and_result(self, value, publisher_name):
        if value and self.bool_value2:
            result = True
        else:
            result = False

        self.get_logger().info(f'{publisher_name} - {value} | pub2 - {self.bool_value2} → {"sudah siap nih, gass min!" if result else "tunggu dulu, kami belum ready!"}')


def main(args=None):
    rclpy.init(args=args)
    alternating_publishers = AlternatingPublishers()
    rclpy.spin(alternating_publishers)
    alternating_publishers.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
