## Onboarding process of custom robot to the system:

Onboarding of a new robot can be done easily by using the bash script provided here. The only thing left is to add the information about the manipulators of the robot manually and your robot is ready to use the 5G-ERA Middleware!

Following is also the robot onboarding template. Although it may look simple, the ROS architecture of the robots can be very complex and therefore is it automated by running a bash script to fetch all that information.

```json
{
  "Id": null,
  "Name": null,
  "ROSRepo": null,
  "ROSNodes": [],
  "Manufacturer": null,
  "ManufacturerUrl": null,
  "RobotModel": null,
  "RobotStatus": null,
  "TaskList": null,
  "BatteryStatus": null,
  "MacAddress": null,
  "LocomotionSystem": null,
  "LocomotionTypes": null,
  "Sensors": [],
  "ActuatorModel": [],
  "Manipulators": [],
  "CPU": null,
  "RAM": null,
  "StorageDisk": null,
  "NumberCores": null,
  "Questions": null,
  "Relations": []
}
```
