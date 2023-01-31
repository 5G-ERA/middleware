# Onboard a new user to the middleware system:

## Register New User

To create a new user in the system use the following endpoint. You will be required to enter a randomly generated GUID and a password. Remember this ones for future log in.Change localhost to the IP of your middleware is you are not working in development enviroment under visual studio. Here, we are using Postman. To create a new user in the system, use the following endpoint. You will be required to enter a randomly generated GUID and a password, and remember the details for future login.        


```
http://localhost:5047/Register
```

![image](https://github.com/5G-ERA/middleware/blob/main/docs/1_Middleware/1_Onboarding/img/RegisterRobot.jpg)

Onboarding of a new Robot requires an onboarding file available for you already. If your robot is in the [list](https://github.com/5G-ERA/middleware/tree/main/docs/1_Middleware/1_Onboarding/AvailableRobots), and adjust to the minor changes. If your robot is not in the list, then you need to generate the onboarding template before. This template is in json format, that needs to be included in the body of the http request. [Download](https://github.com/5G-ERA/middleware/blob/main/util/onboardingRobot.sh) the bash script and run in your robot to get json file. Next step is to prepare your http request using Postman (we are using it here). The endpoint that needs to be called is: 
http://localhost:5047/Data/Robot

## Change the Robot Name 

If the status is invalid, after registering, please try changing the name of the Robot.

![image](https://github.com/5G-ERA/middleware/blob/main/docs/1_Middleware/1_Onboarding/img/Robotnamechange.jpg)



