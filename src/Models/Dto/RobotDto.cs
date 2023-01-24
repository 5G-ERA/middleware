using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Middleware.Models.Dto.Ros;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "robot-idx", StorageType = StorageType.Json, Prefixes = new[] { "Robot" })]
public class RobotDto : Dto
{
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; }
    [Indexed]
    public string? Name { get; set; }
    [Indexed(JsonPath = "$.RosDistro")]
    public RosInfo Ros { get; set; } = new();
    [Indexed(Sortable = true)]
    public DateTimeOffset LastUpdatedTime { get; set; }
    [Indexed]
    public string? RobotModelName { get; set; }
    [Indexed]
    public string? RobotStatus { get; set; }
    [Indexed]
    public string Manufacturer { get; set; }

    public Uri ManufacturerUrl { get; set; }
    [Indexed]
    public long MaximumPayload { get; set; }
    [Indexed]
    public long MaximumTranslationalVelocity { get; set; }

    public long MaximumRotationalVelocity { get; set; }

    public long RobotWeight { get; set; }
    [Indexed]
    public long BatteryStatus { get; set; }

    public string? MacAddress { get; set; }

    public string? LocomotionSystem { get; set; }

    public string? LocomotionType { get; set; } // Compulsory field
    [Indexed]
    public List<Sensor> Sensors { get; set; } = new();
    [Indexed]
    public List<Actuator> Actuators { get; set; } = new();
    [Indexed]
    public List<Manipulator> Manipulators { get; set; } = new();
    [Indexed]
    public List<string> QuestionIds { get; set; } = new();
    public HardwareSpec HardwareSpec { get; set; } = new();

    public override BaseModel ToModel()
    {
        var dto = this;
        return new RobotModel()
        {
            Id = Guid.Parse(dto.Id!),
            Name = dto.Name,
            BatteryStatus = dto.BatteryStatus,
            LocomotionSystem = dto.LocomotionSystem,
            Manufacturer = dto.Manufacturer,
            MacAddress = dto.MacAddress,
            ManufacturerUrl = dto.ManufacturerUrl,
            MaximumPayload = dto.MaximumPayload,
            RobotStatus = dto.RobotStatus,
            RobotWeight = dto.RobotWeight,
            LastUpdatedTime = dto.LastUpdatedTime.DateTime,
            RobotModelName = dto.RobotModelName,
            MaximumRotationalVelocity = dto.MaximumRotationalVelocity,
            MaximumTranslationalVelocity = dto.MaximumTranslationalVelocity,
            Cpu = dto.HardwareSpec.Cpu,
            Ram = dto.HardwareSpec.Ram,
            NumberCores = dto.HardwareSpec.NumberCores,
            StorageDisk = dto.HardwareSpec.StorageDisk,
            LocomotionTypes = dto.LocomotionType,
            RosDistro = dto.Ros.RosDistro,
            RosVersion = dto.Ros.RosVersion,
            //TODO: fill remaining specification of the robot
        };
    }
}