To run the onboarding script, copy the python file to the robot directory you prefer. Make sure the script have running permission.

```
chmod 777
```

Install the program dependences:

```
pip install -r requirements.txt
```

Launch the code with the following statement:

```
python3 onboarding_robot.py
```

The output file will be named **robot_onboarding_v1.json** and it will have the ROS node section completed by quering the ROS for each single node.
