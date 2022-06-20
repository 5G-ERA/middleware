# Usfeul documentation:
# http://design.ros2.org/articles/actions.html#goal-states
# http://design.ros2.org/articles/actions.html
# https://docs.ros2.org/latest/api/rclpy/api/actions.html
# http://wiki.ros.org/actionlib

# ROS Client Library for Python
from typing import List
import rclpy
# Handles the creation of nodes
from rclpy.node import Node
# REST Library for Python
import requests
from requests.exceptions import HTTPError
# JSON format python library
import json
import os
import uuid
import time

# ActionServer library for ROS 2 Python
from rclpy.action import ActionServer
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.action import *

from rclpy.action.server import ServerGoalHandle

from action_tutorials_interfaces.action import Fibonacci
import threading
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
import collections

password = "5g-era"
global Token
Token = ''
global plan
plan = ''
global login_ocelot
login_ocelot = "http://10.64.140.43/Login"
global plan_ocelot
plan_ocelot = "http://10.64.140.43/Task/Plan"
global resourceStatusOcelot
resourceStatusOcelot = "http://10.64.140.43/orchestrate/orchestrate/plan/"
id = "97f4059f-acfa-48f9-b22c-165ecbc51ed1"
global ActionPlanID
ActionPlanID = ""
global PreviousActionId
PreviousActionId = ""


class ActionServer5G(rclpy.node.Node):
    def __init__(self, *args):
        super().__init__('actionClient')  # Defines class of type ActionServer5G and inherits from subclass Node
        self._goal_queue = collections.deque()
        self._goal_queue_lock = threading.Lock()

        self._goal_handle = None
        self._action_server = ActionServer(
            self,
            Fibonacci,
            'fibonacci',
            handle_accepted_callback=self.handle_accepted_callback,
            execute_callback=self.execute_callback,
            goal_callback=self.goal_callback,
            cancel_callback=self.cancel_callback)  # instantiate a new action server

    def destroy(self):
        self._action_server.destroy()
        super().destroy_node()

    def gateway_login(self, id, password):
        self.get_logger().info("Trying to log into the middleware")
        # Request Login
        try:
            r = requests.post(login_ocelot, json={"Id": id, "Password": password})
            # self.get_logger().info(f"Status Code: {r.status_code}, Response: {r.json()}")
            Token = jsondata = r.json()
            newToken = (Token['token'])  # Token is stored here
            return newToken

        except HTTPError as e:
            self.get_logger().ERROR("Could not login to the middleware gateway")
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not login to gateway, revisit the log files for more details.'

    def gateway_get_plan(self, newToken, taskid, plan_ocelot):
        # Request plan
        try:
            print("Goal task is: ", taskid)
            self.get_logger().info("Goal task is : " + taskid)
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            data = {"TaskId": str(taskid)}

            response = requests.post(plan_ocelot, json=data, headers=hed)
            self.get_logger().info("PLAN: " + str(response.json()))
            Action_Sequence = jsondata = response.json()
            actionseq = Action_Sequence['ActionSequence']
            ActionPlanId = Action_Sequence['ActionPlanId']
            self.get_logger().info("ActionPlanId ** is: " + str(actionseq))
            return response.json(), ActionPlanId

        except HTTPError as e:
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not get the plan, revisit the log files for more details.'

    def gateway_get_number_steps_in_plan(self, newToken: str, taskid) -> int:  # Get number of steps in plan
        try:
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            data = {"TaskId": taskid}
            response = requests.post(plan_ocelot, json=data, headers=hed)
            Action_Sequence = jsondata = response.json()
            actionseq = (Action_Sequence['ActionSequence'])
            return (len(actionseq))
        except HTTPError as e:
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not get the number of steps, revisit the log files for more details.'

    def gateway_get_actionSequenceIds(self, plan: dict) -> List:
        try:
            self.get_logger().info("type "+str(type(plan)))
            Action_Sequence_Data = plan['ActionSequence']
            number_steps = len(Action_Sequence_Data)
            ActionSequenceIdsList = []
            for x in range(0,
                           number_steps):  # Iterate over all the actions within the action sequence and get their ids.
                Action_SequenceFullData = Action_Sequence_Data[x]
                ActionSequenceIdsList.append(Action_SequenceFullData['Id'])

            return ActionSequenceIdsList  # Return a list of the actions ids.
        except HTTPError as e:
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not get the number list of action ids, revisit the log files for more details.'

    def deleteAllResources(self, newToken, ActionPlanId):
        try:
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            url = "http://10.64.140.43/orchestrate/orchestrate/plan/" + str(ActionPlanId)
            response = requests.delete(url, headers=hed)

            if '200' in str(response):
                self.get_logger().info('Resource deleted with id: ' + id)


        except HTTPError as e:
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not get delete the resource, revisit the log files for more details.'

    def deleteSingleResource(self):
        pass

    def getResourceStatus(self, ActionPlanId, newToken):
        try:  # query orchestrator for latest information regarding the status of resources.
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            url = resourceStatusOcelot + str(ActionPlanId)
            response = requests.get(url, headers=hed)
            # self.get_logger().info('Feedback: {0}'.format("Resource status: "+str(response.json())))
            return str(response.json())
        except HTTPError as e:
            self.get_logger().ERROR(e.response.status_code)
            return 'Error, could not get the resource status, revisit the log files for more details.'

    def gatewayLogOff(self):
        self.get_logger().info('Middleware log out succesful ')
        # needs to be completed asap.
        pass

    def handle_accepted_callback(self, goal_handle):  # Add goal to listi
        self.get_logger().info("-------------------------> Handle accepted goal")
        with self._goal_queue_lock:
            if self._goal_handle is not None and self._goal_handle.is_active:
                self.get_logger().info('Aborting previous goal')
                self._goal_handle.abort()
            self._goal_handle = goal_handle
        # self._current_goal.execute()
        goal_handle.execute()

    def goal_callback(self, goal_handle):  # This function will decide if a goal is accepted or rejected.
        
        self.get_logger().info("---------------------------> RECEIVED GOAL REQUEST - Trying to process request")
        global Token
        global ActionPlanID
        global plan

        index  = goal_handle.action_reference
        self.get_logger().info('Feedback: {0}'.format("index: "+str(index)))

        if (index != 0) and (len(plan) !=0):  # received update for the task
            return GoalResponse.ACCEPT
        
        
        newToken = self.gateway_login(id, password)  # Login to gateway function
        Token = newToken
        
        if 'Error' in newToken:
            self.get_logger().info('Feedback: {0}'.format(
                "Error, could not login to gateway, revisit the log files for more details."))  # Update feedback disply in command line
            return GoalResponse.REJECT
        else:
            self.get_logger().info('Feedback: {0}'.format("Login successful, token received"))
            plan, ActionPlanId = self.gateway_get_plan(newToken, goal_handle.goal_taskid,
                                                       plan_ocelot)  # Get the plan by sending the token and TaskId
            global ActionPlanID
            ActionPlanID = ActionPlanId  # Update the global variable actionPlanId
            self.get_logger().info("ActionPlanId is: " + ActionPlanID)
            if 'Error' in plan:
                self.get_logger().info(
                    'Feedback: {0}'.format("Error, could not get the plan, revisit the log files for more details."))
                return GoalResponse.REJECT
            else:
                self.get_logger().info('Feedback: {0}'.format("Got new plan successfully."))
                return GoalResponse.ACCEPT

    def execute_callback(self, goal_handle: ServerGoalHandle):  # Will execute the goal if it was accepted by the register_goal_callback function.
        global FirstActionId

        global PreviousActionId
        global Token
        global ActionPlanID
        global password
        global id
        global plan

        SubactionRef = goal_handle.request.action_reference
        feedback_msg = Fibonacci.Feedback()  # create action instance feedback

        self.get_logger().info('SubactionRef '+str(SubactionRef))
        self.get_logger().info('SubactionRef '+str(type(SubactionRef)))
        if SubactionRef == 0:  # This is a completely new action.
            FirstActionId = SubactionRef
            self.get_logger().info('Executing New action goal... ')

             # actionSequenceIds = self.gateway_get_actionSequenceIds(Token, goal_handle.request.goal_taskid)
            actionSequenceIds = self.gateway_get_actionSequenceIds(plan)
            self.get_logger().info("ActionSequenceIds: " + str(actionSequenceIds))
            feedback_msg.action_sequence = actionSequenceIds
            feedback_msg.feedback_resources_status = ''
            goal_handle.publish_feedback(feedback_msg)  # Publish the feedback
            self.get_logger().info("Publishing first feedback")
            self.get_logger().info("goal handle: " + str(goal_handle))

            # goal_handle.succeed()
            # result.success = True

            while goal_handle.is_active:

                self.get_logger().info("Processing goal")

                if not goal_handle.is_active:
                    #  result = Fibonacci.Result()
                    self.get_logger().info('Goal aborted')
                    # result = 'Goal aborted'
                    # return Fibonacci.Result

                if goal_handle.is_cancel_requested:
                    # result = Fibonacci.Result()
                    # result = 'Goal canceled'
                    goal_handle.canceled()
                    self.get_logger().info('Goal canceled')
                #      goal_handle.canceled()
                # return Fibonacci.Result()

                self.get_logger().info("Looping query reosurce update")
                time.sleep(1)
                feedback_msg.feedback_resources_status = self.getResourceStatus(ActionPlanID,
                                                                                Token)  # Not sure this will work. May need another timer approach.
                actionSequenceIds = self.gateway_get_actionSequenceIds(plan)
                goal_handle.publish_feedback(feedback_msg)  # Publish the feedback
                time.sleep(1)

        elif SubactionRef == 1:  # Goal succesful
           
            # self.get_logger().info('Removing resource allocated to action id: '+actionSequenceIds[index_lentgh])
            # self.deleteResource(actionSequenceIds[index_lentgh])
            self.get_logger().info('Removing resources')
            self.deleteAllResources(Token, ActionPlanID)
            goal_handle.succeed()  # Set the goal to be a success
            resultObject = Fibonacci.Result()
            resultObject.result = "Finished"
            goal_handle.succeed()

        else:  # This is a subaction - check
            self.get_logger().info("Subaction")

            while goal_handle.is_active:
                self.get_logger().info("Looping query reosurce update")
                time.sleep(1)
                feedback_msg.feedback_resources_status = self.getResourceStatus(ActionPlanID,
                                                                                Token)  # Not sure this will work. May need another timer approach.
                actionSequenceIds = self.gateway_get_actionSequenceIds(plan)
                goal_handle.publish_feedback(feedback_msg)  # Publish the feedback
                time.sleep(1)

    def cancel_callback(self, cancel_callback):  # function to be triggered once a goal is canceled by client side.
        global ActionPlanID
        global Token
        self.deleteAllResources(Token, ActionPlanID)  # It will delete all the resources.
        return CancelResponse.ACCEPT


def main(args=None):
    rclpy.init(args=args)

    actionServer = ActionServer5G()
    multi_thread_executor = MultiThreadedExecutor()
    multi_thread_executor.add_node(actionServer)
    rclpy.spin(actionServer, executor=multi_thread_executor)
    rclpy.shutdown()

