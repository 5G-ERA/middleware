# Onboard a new user to the middleware system:

## Register New User

To create a new user in the system use the following endpoint. You will be required to enter a randomly generated GUID and a password. Remember these ones for future log in. Change localhost to the IP of your middleware if you are not working in development enviroment under visual studio. And example of the request body:

```
{
    "Id": "ef7bd354-f891-4a24-8576-4fdc6143f941",
    "Password": "new_user"
}

```

The endpoint for registration:

**Remeber to change localhost to the IP of your middleware**

```
http://localhost:5047/Register
```

![image](img/RegisterRobot.jpg)
