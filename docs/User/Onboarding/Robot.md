## Onboarding process of custom robot to the system:

Onboarding of a new robot requires an Onboarding file that may be avaialable for you already. If your robot is in this [list](https://github.com/5G-ERA/middleware/tree/main/docs/1_Middleware/1_Onboarding/AvailableRobots) you may use this template and just adjust it to the minor changes you may have. If your robot, however is not in the list, you will have to generate the onboarding template before. This template is a json that needs to be included in the body of the http request. [Download](https://github.com/5G-ERA/middleware/blob/main/util/onboardingRobot.sh) the bash script and run it in your robot to get the json file. Following, prepare your http request using postman or your prefered editor. The endpoint you will be calling is:

```
http://localhost:5047/Data/Robot
````

<p align="left">
  <img src="img/OnboardRobot.png" alt="Middleware architecture"/>
</p>

___
The headers required for this request should be the ones indicated in the image below.

<p align="left">
  <img src="img/OnboardingRobotHeaders.png" alt="Middleware architecture"/>
</p>

