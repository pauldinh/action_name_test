import rclpy
from rclpy.action import ActionClient
from rclpy.node import Node

from action_tutorials_interfaces.action import Fibonacci


class FibonacciActionClient(Node):

    def __init__(self, num):
        super().__init__('fibonacci_action_client_{}'.format(num+1))

        self._action_name = '/{}'.format('a'*num)

        self._action_client = ActionClient(
            self,
            Fibonacci,
            self._action_name)
        self.get_logger().info('Client created with action name length of `{}`'.format(len(self._action_name)))

        self.send_goal_future = None
        self.should_shutdown = False
        self.timed_out = False

    def send_goal(self, order):
        goal_msg = Fibonacci.Goal()
        goal_msg.order = order

        timeout = 2
        self.get_logger().info('Sending goal. Calling wait_for_server(timeout_sec=%d) again..' % timeout)
        self.wait_for_server(timeout)
        self.send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self.send_goal_future.add_done_callback(self.goal_response_callback)

    def goal_response_callback(self, future):
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected :(')
            return

        self.get_logger().info('Goal accepted :)')

        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future):
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.sequence))
        if self.should_shutdown:
            rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info('Received feedback: {0}'.format(feedback.partial_sequence))

    def wait_for_server(self, timeout):
        result = self._action_client.wait_for_server(timeout_sec=timeout)
        if result:
            self.get_logger().info('client.wait_for_server(timeout_sec={}) returned `{}` for server length `{}`'.format(timeout, result, len(self._action_name)))
        else:
            self.get_logger().warn('client.wait_for_server(timeout_sec={}) returned `{}` for server length `{}`'.format(timeout, result, len(self._action_name)))
            self.timed_out = True



def main(args=None):
    rclpy.init(args=args)
    executor = rclpy.executors.MultiThreadedExecutor()

    clients = []

    rclpy.logging.get_logger('main').info('Creating action clients..')
    for x in range(96,100):
        fibonacci_action_client = FibonacciActionClient(x)
        clients.append(fibonacci_action_client)
        executor.add_node(fibonacci_action_client)

    # designate last client as the "final" client to shutdown everything when its goal is finished
    clients[-1].should_shutdown = True

    rclpy.logging.get_logger('main').info('Quick client.wait_for_server() to see which ones stall..')
    for client in clients:
        client.wait_for_server(timeout=1)

    rclpy.logging.get_logger('main').info('Calling client.send_goal()..')
    # send goals to demonstrate all clients still communicate, even though wait_for_server() never returns True for some clients
    for client in clients:

        # make last client take longer
        if client.should_shutdown:
            client.send_goal(6)
        else:
            client.send_goal(3)

    executor.spin()

    rclpy.logging.get_logger('main').info('Final tally on which clients timed out on wait_for_server():')
    for client in clients:
        if client.timed_out:
            client.get_logger().warn('client.wait_for_server() timed out? %s' % str(client.timed_out))
        else:
            client.get_logger().info('client.wait_for_server() timed out? %s' % str(client.timed_out))

if __name__ == '__main__':
    main()
