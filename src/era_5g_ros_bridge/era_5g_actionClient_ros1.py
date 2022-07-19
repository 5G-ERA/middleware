#! /usr/bin/env python


### Currently work in progress -19-07-2022##

import rospy
from __future__ import print_function
import actionlib
from actions_tutorial.msg import WashTheDishesAction, WashTheDishesGoal

# Brings in the SimpleActionClient
import actionlib

# Brings in the messages used by the fibonacci action, including the
# goal message and the result message.
import actionlib_tutorials.msg


class ActionClientROS1Node():

    def __init__(self):

        #self._actclient = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)
        self.client = actionlib.SimpleActionClient('fibonacci', actionlib_tutorials.msg.FibonacciAction)
        self.taskId = "11071d4d-d1ae-4e55-8de2-e562c6078277"  # Example of task that the robot wants to execute - make sure the taskid is in redis.

    def feedback_cb(self,msg):
        print('Feedback received:', msg)

    def result(self):
        # Waits for the server to finish performing the action.
        self.client.wait_for_result()

        return self.client.get_result()  # A FibonacciResult

    def call_server(self):
        # Creates the SimpleActionClient, passing the type of the action
        # (FibonacciAction) to the constructor.

        # Waits until the action server has started up and started
        # listening for goals.
        self.client.wait_for_server()

        # Creates a goal to send to the action server.
        goal = actionlib_tutorials.msg.FibonacciGoal((taskId,0)

        # Sends the goal to the action server.
        self.client.send_goal(goal)


def main():
        try:
            clientObject = ActionClientROS1Node()
            rospy.init_node('action_client')
            result = clientObject.call_server()
            feedback_data = clientObject.feedback_cb()
            print("feedback_data: ",feedback_data)
            result_data = clientObject.result()
            print("Result data: ",result_data)

        except rospy.ROSInterruptException as e:
            print('Something went wrong:', e)

if __name__ == '__main__':
    main()