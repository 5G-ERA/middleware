using AutoMapper.Execution;
using Middleware.Common.Enums;
using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class RobotModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; } // Compulsory field

    [JsonPropertyName("ROSRepo")]
    public Uri ROSRepo { get; set; }

    [JsonPropertyName("ROSNodes")]

    public List<ROSNodeModel> ROSNodes { get; set; }

    [JsonPropertyName("Manufacturer")]
    public string Manufacturer { get; set; }

    [JsonPropertyName("ManufacturerUrl")]
    public Uri ManufacturerUrl { get; set; }

    [JsonPropertyName("RobotModel")]
    public string RobotModelName { get; set; }

    [JsonPropertyName("RobotStatus")]
    public string RobotStatus { get; set; }

    [JsonPropertyName("CurrentTaskID")]
    public Guid CurrentTaskId { get; set; }

    [JsonPropertyName("TaskList")]
    public List<string> TaskList { get; set; }

    [JsonPropertyName("BatteryStatus")]
    public long BatteryStatus { get; set; }

    [JsonPropertyName("MacAddress")] 
    public string MacAddress { get; set; }

    [JsonPropertyName("LocomotionSystem")] // Compulsory field
    public string LocomotionSystem { get; set; }

    [JsonPropertyName("LocomotionTypes")]
    public string LocomotionTypes { get; set; } // Compulsory field

    [JsonPropertyName("Sensors")]
    public List<SensorModel> Sensors { get; set; }

    [JsonPropertyName("Actuator")]
    public List<ActuatorModel> Actuator { get; set; }

    [JsonPropertyName("Manipulators")]
    public List<RobotManipulatorModel> Manipulators { get; set; }

    [JsonPropertyName("CPU")]
    public long Cpu { get; set; }

    [JsonPropertyName("RAM")] // Compulsory field
    public long Ram { get; set; }

    [JsonPropertyName("VirtualRam")]
    public long VirtualRam { get; set; }

    [JsonPropertyName("StorageDisk")] // Compulsory field
    public long StorageDisk { get; set; } 

    [JsonPropertyName("NumberCores")] // Compulsory field
    public long NumberCores { get; set; }

    [JsonPropertyName("Questions")]
    public List<DialogueModel> Questions { get; set; }

    /// <summary>
    ///  Onboarding validation of the robot data object.
    /// </summary>
    /// <returns></returns>
    public bool IsValid()
    {
        var valLocomotionSystemsEnum = Enum.GetNames(typeof(RobotLocomotionSystemsEnum)).ToList();
        var valLocomotionTypes = Enum.GetNames(typeof(RobotLocomotionTypes)).ToList();

        if (string.IsNullOrEmpty(NumberCores.ToString())) return false;
        if (string.IsNullOrEmpty(StorageDisk.ToString())) return false;
        if (string.IsNullOrEmpty(Ram.ToString())) return false;
        //if (string.IsNullOrEmpty(MacAddress.ToString())) return false;
        if (string.IsNullOrEmpty(Name.ToString())) return false;
        if (string.IsNullOrEmpty(LocomotionSystem.ToString())) return false;
        if (string.IsNullOrEmpty(LocomotionTypes.ToString())) return false;

        if (!valLocomotionTypes.Contains(LocomotionTypes)) return false;
        if (!valLocomotionSystemsEnum.Contains(LocomotionSystem)) return false;

        //Check sensors validity
        if (Sensors.Any() == true)
        {
            var valSensorTypeEnum = Enum.GetNames(typeof(SensorTypeEnum)).ToList();
            foreach (SensorModel sensor in Sensors)
            {
                if (string.IsNullOrEmpty(sensor.Description.ToString())) return false;
                if (string.IsNullOrEmpty(sensor.Name.ToString())) return false;
               // if (string.IsNullOrEmpty(sensor.SensorLocation.ToString())) return false;
                if (string.IsNullOrEmpty(sensor.Type.ToString())) return false;
                if (!valSensorTypeEnum.Contains(sensor.Type)) return false;
                if (sensor.number <= 0) return false;

            }
        }
        //Check actuators validity
        if (Actuator.Any() == true)
        {
            var valActuatorTypesEnum = Enum.GetNames(typeof(RobotActuatorTypesEnum)).ToList();
            foreach (ActuatorModel actuator in Actuator)
            {
                if (string.IsNullOrEmpty(actuator.Name.ToString())) return false;
                if (string.IsNullOrEmpty(actuator.Name.ToString())) return false;
                if (!valActuatorTypesEnum.Contains(actuator.Type)) return false;
                if (actuator.Dof <= 0) return false;
            }
        }

        //Check Manipulators validity
        if (Manipulators.Any() == true)
        {
            var valActuatorTypesEnum = Enum.GetNames(typeof(RobotActuatorTypesEnum)).ToList();
            foreach (RobotManipulatorModel manipulator in Manipulators)
            {
                if (string.IsNullOrEmpty(manipulator.ActuatorName.ToString())) return false;
                if (string.IsNullOrEmpty(manipulator.dof.ToString())) return false;
                if (manipulator.dof <= 0) return false;
                if (manipulator.number <= 0) return false;
            }
        }
        return true;
    }
}
