import time
import sys

import rclpy
from rclpy.action import ActionServer
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionServer(Node):

    def __init__(self, num):
        super().__init__('fibonacci_action_server_{}'.format(num+1))

        self._action_name = '/{}'.format('a'*num)

        self._action_server = ActionServer(
                self,
                Fibonacci,
                self._action_name,
                self.execute_callback)
        self.get_logger().info('Server created with action name length of `{}`'.format(len(self._action_name)))

    def execute_callback(self, goal_handle):
        self.get_logger().info('Executing goal...')

        feedback_msg = Fibonacci.Feedback()
        feedback_msg.partial_sequence = [0, 1]

        for i in range(1, goal_handle.request.order):
            feedback_msg.partial_sequence.append(
                feedback_msg.partial_sequence[i] + feedback_msg.partial_sequence[i-1])
            self.get_logger().info('Feedback: {0}'.format(feedback_msg.partial_sequence))
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(1)

        goal_handle.succeed()

        result = Fibonacci.Result()
        result.sequence = feedback_msg.partial_sequence
        return result


def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    servers = []
    for x in range(96,100):
        fibonacci_action_server = FibonacciActionServer(x)
        servers.append(fibonacci_action_server)
        executor.add_node(fibonacci_action_server)

    executor.spin()


if __name__ == '__main__':
    main()
