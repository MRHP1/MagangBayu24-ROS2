import rclpy
from rclpy.node import Node
from std_msgs.msg import String
import random

class MathPublisher(Node):
    def __init__(self):
        super().__init__('math_publisher')
        self.publisher = self.create_publisher(String, 'math_problems', 10)
        self.timer = self.create_timer(1, self.publish_problem)

    def publish_problem(self):
        math_problem = self.generate_math_problem()
        self.publisher.publish(String(data=math_problem))
        self.get_logger().info(f'Published: {math_problem}')

    def generate_math_problem(self):
        num1 = random.randint(1, 1000)
        num2 = random.randint(1, 1000)
        num3 = random.randint(1, 1000)
        operators = ['+', '-', '*', '/', '%']
        opr1 = random.choice(operators)
        opr2 = random.choice(operators)
        math_problem = f"{num1} {opr1} {num2} {opr2} {num3}"
        return math_problem

def main(args=None):
    rclpy.init(args=args)
    publisher = MathPublisher()
    rclpy.spin(publisher)
    publisher.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
