#!/usr/bin/env python3 
"""
Description:
  Action Client to connect to 5G-ERA gateway and get token. Post request
-------
Author: Adrian Lendinez
Website: 
Date: April 11, 2022
"""
# Import our action definition
from action_tutorials_interfaces.action import ConnectToChargingDock
 
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



 
class ConnectToChargingDockActionClient(Node):
  """
  Create a ConnectToChargingDockActionClient class, 
  which is a subclass of the Node class.
  """
  def __init__(self):
   
    # Initialize the class using the constructor
    super().__init__('connect_to_charging_dock_action_client')
     
    # Instantiate a new action client
    # self, type of action, action name
    self._action_client = ActionClient(
      self,
      ConnectToChargingDock,
      'connect_to_charging_dock')

  # post request
  api_url = "http://localhost:5047/Login"
  data = {"username": ppp, "password":pp}
  response = requests.post(api_url, json=data)
  response.json()
	
	
  
  def post_request_5g(self,username,password):
	pass
       
  def send_goal(self, desired_battery_state):
    """
    Action client to send the goal
    """ 
    # Set the goal message
    goal_msg = ConnectToChargingDock.Goal() #  --> GET /orchestrate/plan/{plan_id}
    goal_msg.desired_battery_state = desired_battery_state    
    
    # Wait for the Action Server to launch # get address from action server
    self._action_client.wait_for_server()
     
    # Register a callback for when the future is complete
    self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)    
    self._send_goal_future.add_done_callback(self.goal_response_callback) 
     
  def goal_response_callback(self, future): 
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
   
  def get_result_callback(self, future):
    """
    Gets the result 
    """
    result = future.result().result
    self.get_logger().info('Result (1 = CHARGING, 3 = NOT CHARGING): {0}'.format(
      result.final_battery_state.power_supply_status))
    rclpy.shutdown()
     
  def feedback_callback(self, feedback_msg):
    feedback = feedback_msg.feedback
    self.get_logger().info(
      'Received feedback (1 = CHARGING, 3 = NOT CHARGING): {0}'.format(
      feedback.current_battery_state.power_supply_status))
 
def main(args=None):
  """
  Entry point for the program.
  """
 
  # Initialize the rclpy library
  rclpy.init(args=args)
   
  # Create the goal message
  desired_battery_state = BatteryState()
  desired_battery_state.voltage = 2.16
  desired_battery_state.percentage = 0.24
  desired_battery_state.power_supply_status = 1
   
  # Create the Action Client node
  action_client = ConnectToChargingDockActionClient()

  # Get token from middleware gateway
  try:

	  action_client.post_request_5g("5gera","ros")

	  if (response.status_code==201):
                self.get_logger().info("Token received")
		pass
          elif (response.status_code==401):
		self.get_logger().info("Unauthorized")
	  else:
	    pass
            self.get_logger().info("Gateway rejected post request")
  except:
   self.get_logger().ERROR("Gateway general error")
    
   
  # Send the goal  
  action_client.send_goal(desired_battery_state)
   
  # Spin to execute callbacks
  rclpy.spin(action_client)
   
if __name__ == '__main__':
    main()
