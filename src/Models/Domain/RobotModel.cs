using System.Text.Json.Serialization;
using System.Xml.Linq;
using Middleware.Models.Dto;
using Middleware.Models.Dto.Hardware;
using Middleware.Models.Dto.Ros;
using Middleware.Models.Enums;

namespace Middleware.Models.Domain;

public class RobotModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; } = Guid.NewGuid();

    [JsonPropertyName("Name")]
    public override string Name { get; set; } // Compulsory field

    [JsonPropertyName("RosVersion")]
    public int RosVersion { get; set; } // Compulsory field

    [JsonPropertyName("RosDistro")]
    public string? RosDistro { get; set; }

    [JsonPropertyName("MaximumPayload")]
    public long? MaximumPayload { get; set; }

    [JsonPropertyName("MaximumTranslationalVelocity")]
    public long? MaximumTranslationalVelocity { get; set; }

    [JsonPropertyName("MaximumRotationalVelocity")]
    public long? MaximumRotationalVelocity { get; set; }

    [JsonPropertyName("RobotWeight")]
    public long? RobotWeight { get; set; }

    [JsonPropertyName("ROSRepo")]
    public Uri? ROSRepo { get; set; }

    [JsonPropertyName("ROSNodes")]

    public List<ROSNodeModel>? ROSNodes { get; set; }

    [JsonPropertyName("Manufacturer")]
    public string? Manufacturer { get; set; }

    [JsonPropertyName("ManufacturerUrl")]
    public Uri? ManufacturerUrl { get; set; }

    [JsonPropertyName("RobotModelName")]
    public string? RobotModelName { get; set; }

    [JsonPropertyName("RobotStatus")]
    public string? RobotStatus { get; set; }

    //[JsonPropertyName("CurrentTaskID")]
    public Guid CurrentTaskId { get; set; }

    [JsonPropertyName("TaskList")]
    public List<string>? TaskList { get; set; }

    [JsonPropertyName("BatteryStatus")]
    public int BatteryStatus { get; set; }

    [JsonPropertyName("MacAddress")]
    public string? MacAddress { get; set; }

    [JsonPropertyName("LocomotionSystem")] // Compulsory field
    public string? LocomotionSystem { get; set; }

    [JsonPropertyName("LocomotionType")]
    public string? LocomotionType { get; set; } // Compulsory field

    [JsonPropertyName("Sensors")]
    public List<SensorModel>? Sensors { get; set; }

    [JsonPropertyName("Actuators")]
    public List<ActuatorModel>? Actuators { get; set; }

    [JsonPropertyName("Manipulators")]
    public List<ManipulatorModel>? Manipulators { get; set; }

    [JsonPropertyName("CPU")]
    public int? Cpu { get; set; }

    [JsonPropertyName("RAM")] // Compulsory field
    public long? Ram { get; set; }

    [JsonPropertyName("StorageDisk")] // Compulsory field
    public long? StorageDisk { get; set; }

    [JsonPropertyName("NumberCores")] // Compulsory field
    public int? NumberCores { get; set; }

    [JsonPropertyName("Questions")]
    public List<DialogueModel>? Questions { get; set; }

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
        var valLocomotionSystemsEnum = Enum.GetNames(typeof(RobotLocomotionSystem)).ToList();
        var valLocomotionTypes = Enum.GetNames(typeof(RobotLocomotionType)).ToList();

        if (string.IsNullOrEmpty(NumberCores.ToString())) return false;
        if (string.IsNullOrEmpty(StorageDisk.ToString())) return false;
        if (string.IsNullOrEmpty(Ram.ToString())) return false;
        //if (string.IsNullOrEmpty(MacAddress.ToString())) return false;
        if (string.IsNullOrEmpty(Name.ToString())) return false;
        if (string.IsNullOrEmpty(LocomotionSystem.ToString())) return false;
        if (string.IsNullOrEmpty(LocomotionType.ToString())) return false;

        if (!valLocomotionTypes.Contains(LocomotionType)) return false;
        if (!valLocomotionSystemsEnum.Contains(LocomotionSystem)) return false;

        //Check sensors validity
        if (Sensors.Any() == true)
        {
            var valSensorTypeEnum = Enum.GetNames(typeof(SensorType)).ToList();
            foreach (SensorModel sensor in Sensors)
            {
                if (string.IsNullOrEmpty(sensor.Description.ToString())) return false;
                if (string.IsNullOrEmpty(sensor.Name.ToString())) return false;
                // if (string.IsNullOrEmpty(sensor.SensorLocation.ToString())) return false;
                if (string.IsNullOrEmpty(sensor.Type.ToString())) return false;
                if (!valSensorTypeEnum.Contains(sensor.Type)) return false;
                if (sensor.Number <= 0) return false;
            }
        }

        //Check actuators validity
        if (Actuators.Any() == true)
        {
            var valActuatorTypesEnum = Enum.GetNames(typeof(RobotActuatorType)).ToList();
            foreach (ActuatorModel actuator in Actuators)
            {
                if (string.IsNullOrEmpty(actuator.Name.ToString())) return false;
                if (string.IsNullOrEmpty(actuator.Name.ToString())) return false;
                if (!valActuatorTypesEnum.Contains(actuator.Type)) return false;
            }
        }

        //Check Manipulators validity
        if (Manipulators.Any() == true)
        {
            var valActuatorTypesEnum = Enum.GetNames(typeof(RobotActuatorType)).ToList();
            foreach (ManipulatorModel manipulator in Manipulators)
            {
                if (string.IsNullOrEmpty(manipulator.ActuatorName.ToString())) return false;
                if (string.IsNullOrEmpty(manipulator.Dof.ToString())) return false;
                if (manipulator.Dof <= 0) return false;
                if (manipulator.Number <= 0) return false;
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
    public RosTopicModel? GetTopicModelFromRobot(string topicName)
    {
        return GetAllRobotTopics().Where(t => t.Name == topicName).FirstOrDefault();
    }
    
    public override Dto.Dto ToDto()
    {

        var domain = this;
        return new RobotDto()
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            Ros = new RosInfo()
            {
                RosDistro = domain.RosDistro,
                RosNodes = domain.ROSNodes?.Select(x=>x.ToDto()).ToList(),
                RosRepo = domain.ROSRepo,
                RosVersion = domain.RosVersion
            },
            MaximumPayload = domain.MaximumPayload,
            MaximumTranslationalVelocity = domain.MaximumTranslationalVelocity,
            MaximumRotationalVelocity = domain.MaximumRotationalVelocity,
            RobotWeight = domain.RobotWeight,
            Manufacturer = domain.Manufacturer,
            ManufacturerUrl = domain.ManufacturerUrl,
            RobotModelName = domain.RobotModelName,
            RobotStatus = domain.RobotStatus,
            CurrentTaskId = domain.CurrentTaskId.ToString(),
            TaskList = domain.TaskList,
            BatteryStatus = domain.BatteryStatus,
            MacAddress = domain.MacAddress,
            LocomotionSystem = domain.LocomotionSystem,
            LocomotionTypes = domain.LocomotionType,
            Sensors = domain.Sensors?.Select(x => x.ToDto()).ToList(),
            Actuators = domain.Actuators?.Select(x => x.ToDto()).ToList(),
            Manipulators = domain.Manipulators?.Select(x => x.ToDto()).ToList(),
            HardwareSpec = new HardwareSpec()
            {
              Cpu  = domain.Cpu,
              Ram = domain.Ram,
              NumberCores = domain.NumberCores,
              StorageDisk = domain.StorageDisk
            },
            Questions = domain.Questions,
            LastUpdatedTime = domain.LastUpdatedTime == default ? DateTimeOffset.Now : domain.LastUpdatedTime,
            OnboardedTime = domain.OnboardedTime == default ? DateTimeOffset.Now : domain.OnboardedTime
        };
    }
}
