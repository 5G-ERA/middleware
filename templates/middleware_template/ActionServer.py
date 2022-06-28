#!/usr/bin/env python3
"""
Description:
  Action Server to connect to 5G ERA gateway.
  Action is successful when the power_supply_status goes from
  NOT_CHARGING(i.e. 3) to CHARGING(i.e. 1)
-------
Subscription Topics:
  Current battery state
  /battery_status – sensor_msgs/BatteryState
-------
Publishing Topics:
  Velocity command to navigate to the charging dock.
  /cmd_vel - geometry_msgs/Twist
-------
Author: Adrian Lendinez
Website: A
Date: 11 Abril 2022
"""

#Active/Down/Instantiating/Idle/Terminating/Problem
import time

# Import our action definition
from ActionInterfacde5G.action import task

# ROS Client Library for Python
import rclpy

# ActionServer library for ROS 2 Python
from rclpy.action import ActionServer

# Enables publishers, subscribers, and action servers to be in a single node
from rclpy.executors import MultiThreadedExecutor

# Handles the creation of nodes
from rclpy.node import Node

# Handle Twist messages, linear and angular velocity
from geometry_msgs.msg import Twist
import os


# Rest python library
import requests

from ActionInterfacde5G.action import task

global Token
Token = ''
global plan
plan = ''


# Holds the current state of the battery
this_battery_state = BatteryState()

class ConnectToGatewayActionServer(Node):
  """
  Create a ConnectToGatewayActionServer class,
  which is a subclass of the Node class.
  """
  def __init__(self):

    # Initialize the class using the constructor
    super().__init__('connect_to_gateway_action_server')

    # Instantiate a new action server
    # self, type of action, action name, callback function for executing goals
    self._action_server = ActionServer(
      self,
      ConnectToGateway,
      'connect_to_gateway',
      self.execute_callback)


    # Create a service to get token and plan_id
     self.cli = self.create_client(Plan_and_token, 'add_two_ints')
     while not self.cli.wait_for_service(timeout_sec=1.0):
         self.get_logger().info('service not available, waiting again...')
     self.req = Plan_and_token.Request()

    def send_request(self):
        #self.req.a = int(sys.argv[1])
        self.future = self.cli.call_async(self.req)


  def execute_callback(self, goal_handle):
    """
    Action server callback to execute accepted goals.
    """
    self.get_logger().info('Executing goal...')

    # Interim feedback
    feedback_msg = task.Feedback() #send plan
    feedback_msg.resource_feedback = 'Not ready'

    # For every 1 minute , get feedback
    while feedback_msg.resource_feedback != 'Ready':

          # perform the http query to endpoint with token
          try:

              # get request
              api_url = "http://localhost:5047/GET/orchestrate/plan/{plan_id}"
              jsonResponse = requests.post(api_url, json=data)

              for key, value in jsonResponse.items():
                 print(key, ":", value)
                 # global Token
                 #Token = value

                #data = response.json()
                #data = json.loads(data)
               if (jsonResponse.status_code==201):
                         self.get_logger().info("Received update from orchestrator")

          except HTTPError as e:
              self.get_logger().ERROR(e.response.status_code)

          # Publish the current battery state
          feedback_msg.resource_feedback = value
          self.get_logger().info(value)
          goal_handle.publish_feedback(feedback_msg)


          time.sleep(0.1)

    # Update feedback
    feedback_msg.resource_feedback = resource_feedback
    goal_handle.publish_feedback(feedback_msg)

    # Indicate the goal was successful
    goal_handle.succeed()response
    self.get_logger().info('Resources alocated')
    self.get_logger().info('Successfully deployed services')

    # Create a result message of the action type
    result = ConnectToChargingDock.Result()

    # Update the final battery state
    result.final_battery_state = resource_feedback

    return result


def main(args=None):
  """
  Entry point for the program.
  """

  # Initialize the rclpy library
  rclpy.init(args=args)

  try:

    # Create the Action Server node
    connectToGatewayActionServer = ConnectToGatewayActionServer()

    # send request to get plan and token
    connectToGatewayActionServer.send_request()
    while rclpy.ok():
        rclpy.spin_once(connectToGatewayActionServer)
        # See if the service has replied
        if connectToGatewayActionServer.future.done():
            try:
                Token_param, plan_param = connectToGatewayActionServer.future.result()
                global Token
                Token = Token_param
                global Plan
                Plan = plan_param
            except Exception as e:
                connectToGatewayActionServer.get_logger().info(
                    'Service call failed %r' % (e,))

            break


    # Set up mulithreading
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(ActionServer)


    try:
      # Spin the nodes to execute the callbacks
      executor.spin()
    finally:
      # Shutdown the nodes
      executor.shutdown()
      connectToGatewayActionServer.destroy_node()

  finally:
    # Shutdown
    rclpy.shutdown()

if __name__ == '__main__':
    main()
