#!/usr/bin/env python3
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist
from my_robot_interfaces.srv import DrawSquare
from my_robot_interfaces.action import DrawSquare as DrawSquareAction
from rclpy.action import ActionServer
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import math
import time


class DrawSquareNode(Node):
  def __init__(self):
    super().__init__("draw_square")

    #parameters
    self.declare_parameter("default_side_length", 2.0)
    self.declare_parameter("default_velocity", 0.5)
    self.default_side_length = self.get_parameter("default_side_length").value
    self.default_velocity = self.get_parameter("default_velocity").value

    #publisher
    self.publisher_ = self.create_publisher(Twist, "/turtle1/cmd_vel", 10)

    cb_group = ReentrantCallbackGroup()

    #service
    self.service_ = self.create_service(
      DrawSquare,
      "draw_square_service",
      self.draw_square_service_callback,
      callback_group=cb_group,
    )

    #action
    self._action_server = ActionServer(
      self,
      DrawSquareAction,
      "draw_square_action",
      self.draw_square_action_callback,
      callback_group=cb_group,
    )

    self.get_logger().info("Draw Square Node has been started")

  def draw_square_service_callback(self, request, response):
    """Service callback function that gets called when the service is invoked"""
    self.get_logger().info("Received service request to draw a square")
    side_length = (
      request.side_length if request.side_length > 0 else self.default_side_length
    )
    velocity = request.velocity if request.velocity > 0 else self.default_velocity

    success = self.draw_square(side_length, velocity)

    response.success = success
    response.message = (
      "Square drawing completed" if success else "Failed to draw square"
    )
    return response

  def draw_square_action_callback(self, goal_handle):
    """Action callback function that gets called when the action is invoked (simplified)"""
    self.get_logger().info("Received action request to draw a square")

    #feedback
    feedback_msg = DrawSquareAction.Feedback()
    result_msg = DrawSquareAction.Result()

    side_length = (
      goal_handle.request.side_length
      if goal_handle.request.side_length > 0
      else self.default_side_length
    )
    velocity = (
      goal_handle.request.velocity
      if goal_handle.request.velocity > 0
      else self.default_velocity
    )

    self.get_logger().info(
      f"Drawing square with side_length={side_length}, velocity={velocity}"
    )
    feedback_msg.side_completed = 0
    feedback_msg.percent_complete = 0.0
    goal_handle.publish_feedback(feedback_msg)

    #draw
    success = self.draw_square(side_length, velocity)

    feedback_msg.side_completed = 4
    feedback_msg.percent_complete = 100.0
    goal_handle.publish_feedback(feedback_msg)

    #result
    result_msg.success = success
    result_msg.message = (
      "Square drawing completed successfully"
      if success
      else "Failed to draw square"
    )

    if success:
      goal_handle.succeed()
    else:
      goal_handle.abort()

    return result_msg

  def draw_square(self, side_length=2.0, velocity=0.5):
    """Draw a square with the given side length and velocity"""
    #Twist message
    twist = Twist()
    move_time = side_length / velocity

    for i in range(4):
      #move forward
      self.get_logger().info(f"Moving forward (side {i+1})")
      twist.linear.x = velocity
      twist.angular.z = 0.0
      self.publisher_.publish(twist)
      time.sleep(move_time)

      #stop
      twist.linear.x = 0.0
      self.publisher_.publish(twist)
      time.sleep(0.5)

      #turn pi/2
      self.get_logger().info(f"Turning 90 degrees")
      twist.angular.z = math.pi / 2
      self.publisher_.publish(twist)
      time.sleep(1.0)

      #stop
      twist.angular.z = 0.0
      self.publisher_.publish(twist)
      time.sleep(0.5)

    self.get_logger().info("Square completed!")
    return True


def main(args=None):
  rclpy.init(args=args)
  node = DrawSquareNode()

  # Use multi-threaded executor for service and action servers
  executor = MultiThreadedExecutor()
  executor.add_node(node)

  executor.spin()
  executor.shutdown()
  node.destroy_node()
  rclpy.shutdown()


if __name__ == "__main__":
  main()
