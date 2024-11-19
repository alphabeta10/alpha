import time

import rclpy

from rclpy.node import Node
from rclpy.action import ActionServer,ActionClient
from learning_interface.action import MoveCircle


class MoveCircleActionServer(Node):

    def __init__(self,name):
        super().__init__(name)
        self._action_server = ActionServer(
            self,MoveCircle,
            "move_circle",
            self.execute_callback
        )
    def execute_callback(self,goal_handle):
        self.get_logger().info("Moving circle...")
        feedback_msg = MoveCircle.Feedback()

        for i in range(0,360,30):
            feedback_msg.state = i
            self.get_logger().info("Publishing feedback:%d"%i)
            goal_handle.publish_feedback(feedback_msg)
            time.sleep(0.5)

        goal_handle.succeed()
        result = MoveCircle.Result()
        result.finish = True
        return result

def action_server_main(args=None):
    rclpy.init(args=args)
    node = MoveCircleActionServer("action_move_server")
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()

class MoveCircleActionClient(Node):

    def __init__(self,name):
        super().__init__(name)
        self._action_client = ActionClient(
            self,MoveCircle,"move_circle"
        )

    def send_goal(self,enable):

        goal_msg = MoveCircle.Goal()
        goal_msg.enable = enable

        self._action_client.wait_for_server()
        self._send_goal_future = self._action_client.send_goal_async(
            goal_msg,
            feedback_callback=self.feedback_callback
        )
        self._send_goal_future.add_done_callback(
            self.goal_response_callback
        )

    def feedback_callback(self,feedback_msg):
        feedback = feedback_msg.feedback
        self.get_logger().info("Received feedback:{%d}"%feedback.state)
    def goal_response_callback(self,future):

        goal_handle = future.result()

        if not goal_handle.accepted:
            self.get_logger().info("Goal rejected :(")

        self.get_logger().info("Goal accepted:)")

        self._get_result_future = goal_handle.get_result_async()  # 异步获取动作最终执行的结果反馈
        self._get_result_future.add_done_callback(self.get_result_callback)  # 设置一个收到最终结果的回调函数

    def get_result_callback(self, future):  # 创建一个收到最终结果的回调函数
        result = future.result().result  # 读取动作执行的结果
        self.get_logger().info('Result: {%d}' % result.finish)  # 日志输出执行结果


def action_client_main(args=None):
    rclpy.init(args=args)
    node = MoveCircleActionClient("action_move_client")
    node.send_goal(True)
    rclpy.spin(node)
    node.destroy_node()
    rclpy.shutdown()







