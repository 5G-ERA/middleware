# ROS Client
import rclpy
from rclpy.node import Node
# ActionServer library for ROS 2 Python
from rclpy.action import ActionClient
from action_tutorials_interfaces.action import Fibonacci
import time

taskId = "11071d4d-d1ae-4e55-8de2-e562c6078277" #Example of task that the robot wants to execute - make sure the taskid is in redis.
global actionSequence
actionSequence = []
global resourceStatus
resourceStatus = ''

class ActionClient5G(Node):
    def __init__(self):
        super().__init__('actionClient')  # Defines class of type ActionServer5G and inherits from subclass Node
        self._action_client  = ActionClient(self, Fibonacci, 'fibonacci')  # instantiate a new action server

    def send_goal(self, order,ref): # send an action goal message 'canlelation request' / success remove resources
        goal_msg = Fibonacci.Goal()
        goal_msg.goal_taskid = order #task id
        goal_msg.action_reference = ref # Action reference
        self.get_logger().info("Waiting for action server")
        self._action_client.wait_for_server()
        self.get_logger().info("Sending goal request")
        self._send_goal_future = self._action_client.send_goal_async(goal_msg, feedback_callback=self.feedback_callback)
        self._send_goal_future.add_done_callback(self.goal_response_callback)


    def goal_response_callback(self, future): # Method to handle what to do after goal was either rejected or accepted.
        goal_handle = future.result()
        if not goal_handle.accepted:
            self.get_logger().info('Goal rejected!')
            return


        self.get_logger().info('Goal accepted! ')
        self._get_result_future = goal_handle.get_result_async()
        self._get_result_future.add_done_callback(self.get_result_callback)

    def get_result_callback(self, future): # Method to handle what to do after receiving the action result
        result = future.result().result
        self.get_logger().info('Result: {0}'.format(result.result))
        rclpy.shutdown()

    def feedback_callback(self, feedback_msg):
        global actionSequence
        global resourceStatus

        feedback = feedback_msg.feedback
        listActions = feedback.action_sequence # get action sequence
        actionSequence = listActions # update global variable
        resource_status = feedback.feedback_resources_status #get resource feedback for specific action id
        resourceStatus = resource_status # update global variable

        self.get_logger().info('Received feedback: {0}'.format(resource_status))
        self.get_logger().info('Action plan: ' + str(actionSequence))

    def cancel_callback(self,goal_handle):
        self.get_logger().info("Canceling goal...")



def main(args=None):
    rclpy.init(args=args)
    global actionSequence #can be accesed from here
    global resourceStatus #can be accesed fron here


    actionClient = ActionClient5G()

    future = actionClient.send_goal(taskId,0) #send an action goal


    #actionClient.send_goal(taskId,1)
    #Iterate over the action sequence
    #if (len(actionSequence)) == 1:
     #   time.sleep(5)
      #  actionClient.send_goal(taskId,1)

    #for x in range(0,len(actionSequence)):

     #   actionClient.send_goal(taskId, actionSequence[x])
      #  time.sleep(1)#sleep to simulate executing task.
        # cancel_goal() --> if we want to cancel the goal

    #actionClient.send_goal(taskId, 1)
    rclpy.spin(actionClient)


if __name__ == '__main__':
    main()