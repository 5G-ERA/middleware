using System.Text.Json.Serialization;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class RobotModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; } // Compulsory field

    [JsonPropertyName("RosVersion")]
    public int RosVersion { get; set; } // Compulsory field

    [JsonPropertyName("RosDistro")]
    public string RosDistro { get; set; }

    [JsonPropertyName("MaximumPayload")]
    public long MaximumPayload { get; set; }

    [JsonPropertyName("MaximumTranslationalVelocity")]
    public long MaximumTranslationalVelocity { get; set; }

    [JsonPropertyName("MaximumRotationalVelocity")]
    public long MaximumRotationalVelocity { get; set; }

    [JsonPropertyName("RobotWeight")]
    public long RobotWeight { get; set; }

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

    //[JsonPropertyName("CurrentTaskID")]
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

    //  [JsonPropertyName("VirtualRam")]
    //  public long VirtualRam { get; set; }

    [JsonPropertyName("StorageDisk")] // Compulsory field
    public long StorageDisk { get; set; }

    [JsonPropertyName("NumberCores")] // Compulsory field
    public long NumberCores { get; set; }

    [JsonPropertyName("Questions")]
    public List<DialogueModel> Questions { get; set; }

    [JsonPropertyName("LastUpdatedTime")]
    public DateTime LastUpdatedTime { get; set; }

    [JsonPropertyName("OnboardedTime")]
    public DateTime OnboardedTime { get; set; }

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
            }
        }

        //Check Manipulators validity
        if (Manipulators.Any() == true)
        {
            var valActuatorTypesEnum = Enum.GetNames(typeof(RobotActuatorTypesEnum)).ToList();
            foreach (RobotManipulatorModel manipulator in Manipulators)
            {
                if (string.IsNullOrEmpty(manipulator.ActuatorName.ToString())) return false;
                if (string.IsNullOrEmpty(manipulator.Dof.ToString())) return false;
                if (manipulator.Dof <= 0) return false;
                if (manipulator.number <= 0) return false;
            }
        }

        return true;
    }


    /// <summary>
    /// Get all topics of a robot in a single list.
    /// </summary>
    /// <param name="nodes"></param>
    /// <returns></returns>
    public HashSet<RosTopicModel> GetAllRobotTopics()
    {
        HashSet<RosTopicModel> topics = new HashSet<RosTopicModel>();
        foreach (ROSNodeModel node in ROSNodes)
        {
            foreach (RosTopicModel pubTopic in node.Publications)
            {
                topics.Add(pubTopic);
            }

            foreach (RosTopicModel subTopic in node.Subscriptions)
            {
                topics.Add(subTopic);
            }
        }

        return topics;
    }

    /// <summary>
    /// Get topicModel entity from specific topic name
    /// </summary>
    /// <param name="topicName"></param>
    /// <returns></returns>
    public RosTopicModel GetTopicModelFromRobot(string topicName)
    {
        return GetAllRobotTopics().Where(t => t.Name == topicName).FirstOrDefault();
    }
    
    public override Dto.Dto ToDto()
    {
        throw new NotImplementedException();
    }
}
