# middleware-actionserver
Action server is responsible for the communication between the robot and the 5G-ERA Middleware. It is the interface between ROS and robot related applications and middleware task and resource orchestration.

## INTERFACE WORKFLOW:
1) A ROS action client (running in the robot) sends an action goal using ROS action message. This goal has 2 properties: taskId, actionReference.
2) A ROS action server (running in the robot) will listen to this goal request. It will first check if the actionReference is 0, meaning that this goal is not a subgoal of and action sequence that is already running.
 
    2.1) If actionReference is 0, the action server will not need to delete any cloud native resources that may have been created for the task sequence as this is a new task that will start with step 1. Taking this into account, the action server will try to login to the middleware and request a plan. If this is successful, the action goal will change state to accepted. The action server will follow by sending the action sequence plan created by the middleware to the action client so the robots knows which steps should be taken to accomplish this task. 

   2.2) The action client will now send a new action goal with the same taskId and the first actionId from the action sequence. This means that the robot wants to start doing the first action within the action sequence.
  
   2.3) The action server receives this new goal and see that the action reference field is not empty. It will try removing any previous resources alocated to this action sequence if this applies. Since this is the first step, the server side will not tell the middleware to delete any resources as there is none. After this, the server will report back to the client with a frequency of 1 minute, the state of the allocated resource to achieve this action from action sequence. This will loop until the action client sends a new goal meaning in wants to jump to next step.

   2.4) The same process will apply until the last action is reached and finish succesfully by robot. In this case a final action goal will be send with the taskid and the actionReference been 1. This 1 will tell the action server the last action was completed, you may remove the resources and call the goal a success. Configure the action result to be set to successful also.
   
 3) The middleware will log of out of the middleware system. 

## Codying Enviroment set-up (windows base systems):
1) Start by downloading git if you dont have it --> https://git-scm.com/downloads
2) Install Pycharm jetbeans --> https://www.jetbrains.com/es-es/pycharm/
3) In the top bar menu, select VCS and choose git. This will prepare the git application for Pycharm.
4) In the top bar menu, select git --> clone --> copy the ssh clone from this repo
5) *You may need credentaials as this is a private repo. Please generate a new ssh key --> https://docs.github.com/es/authentication/connecting-to-github-with-ssh/checking-for-existing-ssh-keys (Remember to run the command: start-ssh-agent.cmd once the key is generated and copy this one "id_rsa.pub" to the settings, ssh keys in your git profile.)
6) With this, you will have a fully syncronize version of the git repo in pycharm. Please note ROS commands will not be recognised by Pycharm, only native python.


## Run the actionClient and Server:
First, download the zip file from this repo and, if using windows, launch WSL Ubuntu and copy the file using this command:

```
cp -r /mnt/d/User/Desktop/middleware-actionserver-main /home/adrian/robotics/ros2
```
In WSL Ubuntu cd to the directory where the Dockerfile is located under the SRC downloaded solution

Run the command to build the image:
```
docker build -t actionImage:lastest .
```
Run the command to create a container of the image type

```
docker run -d -it actionimage:lastest
```

Connect to the docker container
```
docker exec -it 2288d1f92dea /bin/bash
```
Enroll the container in the middleware network (from outside the container)
```
docker network connect middleware_network 2288d1f92dea
```

If you get the following error, please make sure the middleware application is running before hand as it will create the docker network that this docker-compose will engage with.

![image](https://user-images.githubusercontent.com/26432703/165356762-c77e01b0-7af3-4f95-b743-e11972937f06.png)


Navigate to path --> home/dev_src/ and source the workspace.

```
source install/setup.bash
```
Follow by running in 2 different terminals connected to the container, the action Client and the action Server.

```
ros2 run ActionServer5G ActionServerNode
```
![Captura de pantalla 2022-05-23 105217](https://user-images.githubusercontent.com/26432703/169782041-8446ece6-b420-4010-9196-cd5b1ed87bfb.png)

```
ros2 run ActionClient5G ActionClientNode
```
![Captura de pantalla 2022-05-23 105239](https://user-images.githubusercontent.com/26432703/169782108-7740e28a-0efb-442d-9e42-40b511cc72cb.png)


Additionally, if you want to run the action client as a simple ros2 command, you can use:

```
ros2 action send_goal --feedback fibonacci action_tutorials_interfaces/action/Fibonacci "{goal_taskid: "4225e56e-4b68-4372-9d34-66bba1a633b3",action_reference: 0}"
```
