#! /usr/bin/env python

import rospy
import actionlib

from era_5g_action_interfaces_ros1.msg import *
from era_5g_action_interfaces_ros1.msg import goal_5gActionGoal
from era_5g_action_interfaces_ros1.msg import goal_5gFeedback

from actionlib_msgs.msg import *
from actionlib import ActionServer
from actionlib.server_goal_handle import ServerGoalHandle

import requests
from requests.exceptions import HTTPError
import time
import uuid
import os
import json

import collections
import threading

import signal

global PASSWORD
global TOKEN
global PLAN
global LOGIN_OCELOT
global PLAN_OCELOT
global RESOURCE_STATUS_OCELOT
global K8IP

TOKEN = ''
PLAN = ''
LOGIN_OCELOT = ''
PLAN_OCELOT = ''
RESOURCE_STATUS_OCELOT = ''
ID = '97f4059f-acfa-48f9-b22c-165ecbc51ed1'
ACTION_PLAN_ID = ''
PASSWORD = '5g-era'
K8IP = '10.64.140.43' # Change to the middleware deployed IP

class ActionServerNode():
    def __init__(self):
        # Create an instance of simpleActionServer
        self.a_server = actionlib.SimpleActionServer("goal_5g", goal_5gAction, execute_cb=self.execute_cb,auto_start=False)
        # Set the callback to be executed when a goal is received
        #self.a_server.register_goal_callback(self.goal_callback)
        # Set the callback that should be executed when a preempt request is received
        self.a_server.register_preempt_callback(self.preempt_callback)
        # Start the server
        
        
        self.Num_Steps = 0
        self.ActionSeqIds = []
        self.rate = rospy.Rate(1)

        global K8IP
        global LOGIN_OCELOT
        global PLAN_OCELOT
        global RESOURCE_STATUS_OCELOT

        LOGIN_OCELOT = "http://"+str(K8IP)+"/Login"
        PLAN_OCELOT = "http://"+str(K8IP)+"/Task/Plan"
        RESOURCE_STATUS_OCELOT = "http://" + str(K8IP)+"/orchestrate/orchestrate/plan/"

        rospy.loginfo("Starting 5g-era-action-server...")
        self.a_server.start()


    def destroy(self):
        rospy.loginfo("Shutting down Action Server....")
        self.a_server.shutdown()


    def gateway_login(self,ID,PASSWORD):
        rospy.loginfo("Trying to log into the middleware")
        # Request Login
        try:
            r = requests.post(LOGIN_OCELOT, json={"Id": ID, "Password": PASSWORD})
            Token = jsondata = r.json()
            newToken = (Token['token'])  # Token is stored here
            time.sleep(10)
            return newToken

        except HTTPError as e:
            rospy.loginfo("Could not login to the middleware gateway")
            rospy.loginfo(e.response.status_code)
            return 'Error, could not login to gateway, revisit the log files for more details.'

        
    def gateway_get_plan(self,newToken,taskid,PLAN_OCELOT,resource_lock):
        # Request plan
        try:
            rospy.loginfo("Goal task is: "+str(taskid))
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            data = {"TaskId": str(taskid), "LockResourceReUse": resource_lock}

            response = requests.post(PLAN_OCELOT, json=data, headers=hed)
            rospy.loginfo("PLAN: " + str(response.json()))
            Action_Sequence = jsondata = response.json()
            actionseq = Action_Sequence['ActionSequence']
            ACTION_PLAN_ID = Action_Sequence['ActionPlanId']
            rospy.loginfo("ActionPlanId ** is: " + str(actionseq))


            self.Num_Steps = len(actionseq)

            for x in range(0, self.Num_Steps):  # Iterate over all the actions within the action sequence and get their ids.
                Action_SequenceFullData = actionseq[x]
                rospy.loginfo('Subaction task id: '+Action_SequenceFullData['Id'])
                self.ActionSeqIds.append(Action_SequenceFullData['Id'])


            time.sleep(10)
            return response.json(), ACTION_PLAN_ID
        except KeyError as e:
            rospy.loginfo('There was an error while getting a plan.')
            rospy.loginfo(e)
            return 'Error, could not get the plan, revisit the log files for more details.'


    def deleteAllResources(self, newToken, ACTION_PLAN_ID):
        try:
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            url = RESOURCE_STATUS_OCELOT + str(ACTION_PLAN_ID)
            response = requests.delete(url, headers=hed)

            if '200' in str(response):
                rospy.loginfo('Resource deleted with id: ' + ID)

        except HTTPError as e:
            rospy.loginfo(e.response.status_code)
            return 'Error, could not get delete the resource, revisit the log files for more details.'

    def deleteSingleResource(self):
        pass #TODO

    def getResourceStatus(self, ACTION_PLAN_ID, newToken):
        rospy.loginfo('--> Quering middleware for resource status')
        try:  # query orchestrator for latest information regarding the status of resources.
            hed = {'Authorization': 'Bearer ' + str(newToken)}
            url = RESOURCE_STATUS_OCELOT + str(ACTION_PLAN_ID)
            response = requests.get(url, headers=hed)
            return str(response.json())
        except HTTPError as e:
            rospy.loginfo(e.response.status_code)
            return 'Error, could not get the resource status, revisit the log files for more details.'

    def gatewayLogOff(self):
        rospy.loginfo('Middleware log out succesful ')
        #TODO
        pass

    def handler(signum, frame):
        exit(1)
    
# This function will decide if a goal is accepted or rejected.
    def goal_callback(self,goal_param):

        rospy.loginfo("RECEIVED GOAL REQUEST - Trying to process request")
        
        global TOKEN
        global ACTION_PLAN_ID
        global PLAN
        global ID

        index = goal_param.action_reference
        resource_lock = goal_param.resource_lock
        rospy.loginfo('Feedback: {0}'.format("index: "+str(index)))

        if (index == 0):


            # Login to gateway function
            newToken = self.gateway_login(ID, PASSWORD)
            TOKEN = newToken

            if 'Error' in newToken:
                rospy.loginfo('Feedback: {0}'.format(
                    "Error, could not login to gateway, revisit the log files for more details."))  # Update feedback disply in command line
                
            else:
                rospy.loginfo('Feedback: {0}'.format(
                    "Login successful, token received"))
                #rospy.loginfo('Is the goal active: ',self.a_server.is_active())
            
                try:
                    PLAN, ActionPlanId = self.gateway_get_plan(newToken, goal_param.goal_taskid,PLAN_OCELOT,resource_lock)  # Get the plan by sending the token and TaskId
                except:
                    rospy.loginfo('Error getting a new plan, there is probably a deployment already done of this container. The goal will be canceled...')
                    exit(1)

            try:
                ACTION_PLAN_ID = ActionPlanId  # Update the global variable actionPlanId
                rospy.loginfo("Generated ActionPlanId is: " + ACTION_PLAN_ID)
            except:
                rospy.loginfo(
                    'Feedback: {0}'.format("Error, could not get the plan, revisit the log files for more details."))
                #self.a_server.set_preempted()
                exit(1)


            if 'Error' in PLAN:
                rospy.loginfo(
                    'Feedback: {0}'.format("Error, could not get the plan, revisit the log files for more details."))
                exit(1)
                #self.a_server.set_aborted()
            else:
                rospy.loginfo('Feedback: {0}'.format(
                    "Got new plan successfully."))
               # self.a_server.accept_new_goal()

    # Will execute the goal if it was accepted by the register_goal_callback function.
    def execute_cb(self, goal):
        self.goal_callback(goal)
        rospy.loginfo('Running execute_cb')
        global FirstActionId

        global TOKEN
        global ACTION_PLAN_ID
        global PASSWORD
        global ID
        global PLAN

        SubactionRef = goal.action_reference
        feedback_msg = goal_5gFeedback() # create action instance feedback

        rospy.loginfo('Step number: '+str(SubactionRef))
        #rospy.loginfo('SubactionRef '+str(type(SubactionRef)))
        if SubactionRef != -1:  # This is a completely new action.
            FirstActionId = SubactionRef
        
            self.Num_Steps = 0
            
            actionSequenceIds = self.ActionSeqIds
            rospy.loginfo("ActionSequenceIds: " + str(actionSequenceIds))
            feedback_msg.action_sequence = actionSequenceIds
            feedback_msg.feedback_resources_status = ''

            self.a_server.publish_feedback(feedback_msg)  # Publish the feedback

            rospy.loginfo("Publishing feedback")
            self.rate.sleep()
            #rospy.loginfo("goal handle: " + str(goal_handle))

            # goal_handle.succeed()
            # result.success = True
            while self.a_server.is_active:

                rospy.loginfo("Processing goal")

                rospy.loginfo("Looping query reosurce update")
                self.rate.sleep()
                feedback_msg.action_sequence = self.ActionSeqIds
                update_resource = self.getResourceStatus(ACTION_PLAN_ID,TOKEN) 
                rospy.loginfo(update_resource)
                feedback_msg.feedback_resources_status = str(update_resource)  # Not sure this will work. May need another timer approach.
                
                self.a_server.publish_feedback(
                    feedback_msg)  # Publish the feedback
                self.rate.sleep()



        elif SubactionRef == -1:  # if received -1 as second parameter to the action goal, goal was succesful

            rospy.loginfo('Removing resources')
            self.deleteAllResources(TOKEN, ACTION_PLAN_ID)
            resultObject = Goal5g.Result()
            resultObject.result = "Finished"
            self.a_server.succeed()

                
    def preempt_callback(self):
        """
            Callback executed when a preempt request has been received. --> cancel the goal.
        """
        rospy.loginfo("Action preempted - canceling")
        global ACTION_PLAN_ID
        global TOKEN
        # It will delete all the resources.
        self.deleteAllResources(TOKEN, ACTION_PLAN_ID)
        self.a_server.set_preempted()
        self.a_server.set_aborted()

if __name__ == '__main__':
    rospy.loginfo('Running 5g-era-action-server...')
    rospy.init_node("action_server")
    actionServerObj = ActionServerNode()

    rospy.spin()
