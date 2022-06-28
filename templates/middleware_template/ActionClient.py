#!/usr/bin/env python3
"""
Description:
  Action Client to connect to 5G-ERA gateway. Simulates the robot.
-------
Author: Adrian Lendinez
Website:
Date: April 11, 2022

Supporting Material:
# https://automaticaddison.com/how-to-create-an-action-in-ros-2-galactic-advanced/

# https://docs.ros.org/en/foxy/Tutorials/Using-Parameters-In-A-Class-Python.html

# https://automaticaddison.com/how-to-create-a-service-and-client-python-ros2-foxy/

"""
# Import our action definition
from ActionInterfacde5G.action import task

# ROS Client Library for Python
import rclpy

# ActionClient library for ROS 2 Python
from rclpy.action import ActionClient

# Handles the creation of nodes
from rclpy.node import Node

# Handles BatteryState message
from sensor_msgs.msg import BatteryState

# Rest python library
import requests
from requests.exceptions import HTTPError
import json
import os

global username
username = os.environ.get('ROS_ROBOT_ID')
global password
password = ''
global Token
Token = ''
global plan
plan = ''
global task_id_parm 
task_id_parm =''


from MinimalTask import MinimalTaskModel

class ActionClient5G(Node):
  """
  Create an ActionServer5G class,
  which is a subclass of the Node class.
  """
  def __init__(self):

    # Initialize the class using the constructor
    super().__init__('action_client5g')
    # Declare entry point parameters to the ActionClient - TaskID
    self.declare_parameter('my_parameter', 'TaskIdParam')
    # Get the parameter from command line argument.
    task_id_parameter = self.get_parameter('my_parameter').get_parameter_value().string_value
    global task_id_parm
    task_id_parm = task_id_parameter
    # Print which Task is going to be perform
    self.get_logger().info('Will perform task with ID: %s!' % task_id_parameter)

    # Instantiate a new action client
    # self, type of action, action name
    self._action_client = ActionClient(
      self,
      actionCllient5g,
      'action_client5g')

     # Register en 5G Era middleware via gateway using post request
     try:
         global username
         global password
         # post request
         api_url = "http://localhost:5047/Login"
         data = {"username": username, "password":password}
         jsonResponse = requests.post(api_url, json=data)

         json = jsonResponse.json()
         global Token
         Token = json["token"]


         for key, value in jsonResponse.json().items():
            print(key, ":", Token)
            # global Token
            #Token = value

           #data = response.json()
           #data = json.loads(data)
          if (jsonResponse.status_code==201):
                    self.get_logger().info("Token received: ",Token)

     except HTTPError as e:
         self.get_logger().ERROR(e.response.status_code)

    # Post request to ask for a plan for task_ID
    try:
        # post request
        api_url = "http://localhost:5047/GET/PLAN/"

        my_headers = {'Authorization' : 'Bearer {'+Token+'}'}
        response = requests.get(api_url, headers=my_headers)

        data = {"TaskId": task_id_parameter} #header to request
        jsonResponse = requests.post(api_url, json=data)

        json = jsonResponse.json()
        global plan
        plan = json["ActionPlanId"]
           # global Token
           #Token = value

          #data = response.json()
          #data = json.loads(data)
         if (jsonResponse.status_code==201):
                   self.get_logger().info("Plan received: ",plan)

    except HTTPError as e:
        self.get_logger().ERROR(e.response.status_code)


        # ros service to send token to action server.
     self.srv = self.create_service(Plan_and_token, 'get_plan_and_token', self.send_plan_and_token)

     def send_plan_and_token(self, request, response):
        # Receive the request data and sum it
        response = (Token, plan)

        # Return the sum as the reply
        self.get_logger().info('Incoming request to get Token and TaskId')
        return response

  def send_goal(self, desired_currentTask):
    """
    Action client to send the goal
    """
    # Set the goal message
    goal_msg = task.Goal() #
    goal_msg.desired_goal = desired_currentTask

    # Wait for the Action Server to launch # get address from action server
    self._action_client.wait_for_server()

    # Register a callback for when the future is complete
    self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
    self._send_goal_future.add_done_callback(self.goal_response_callback)

  def goal_response_callback(self, future): #goal callback
    """
    Get the goal_handle
    """
    goal_handle = future.result()
    if not goal_handle.accepted:
      self.get_logger().info('Goal rejected...')
      return

    self.get_logger().info('Goal accepted...')

    self._get_result_future = goal_handle.get_result_async()
    self._get_result_future.add_done_callback(self.get_result_callback)

  def get_result_callback(self, future): # result callback
    """
    Gets the result
    """
    result = future.result().result
    self.get_logger().info('Result: {0}'.format(result.resource_final))
    rclpy.shutdown()

  def feedback_callback(self, feedback_msg): # feedback callback
    feedback = feedback_msg.feedback
    self.get_logger().info('Received feedback: {0}'.format(feedback.resource_feedback))


def main(args=None):
  """
  Entry point for the program.
  """

  # Initialize the rclpy library
  rclpy.init(args=args)


  # Create the Action Client node
  action_client = ActionClient5G()

 # Create the goal message
  CurrentTask = MinimalTaskModel()
  CurrentTask.TaskId = task_id_parm
  CurrentTask.TaskPriority = 'low'
  CurrentTask.status='not_initialize'

  # Send the goal
  action_client.send_goal(CurrentTask)

  # Spin to execute callbacks
  rclpy.spin(action_client)

if __name__ == '__main__':
    main()
