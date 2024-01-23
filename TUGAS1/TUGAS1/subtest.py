import rclpy
from rclpy.node import Node
from std_msgs.msg import String

class MathSubscriber(Node):
    def __init__(self):
        super().__init__('math_subscriber')
        self.subscription = self.create_subscription(
            String, 'math_problems', self.evaluate_problem, 10)
        self.subscription  # prevent unused variable warning

    def evaluate_problem(self, msg):
        math_problem = msg.data
        try:
            result = eval(math_problem)
            self.get_logger().info(f'Evaluated: {math_problem} = {result}')
        except Exception as e:
            self.get_logger().error(f'Error evaluating {math_problem}: {e}')

def main(args=None):
    rclpy.init(args=args)
    subscriber = MathSubscriber()
    rclpy.spin(subscriber)
    subscriber.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
