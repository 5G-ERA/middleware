using Middleware.DataAccess.Dto.Ros;
using Redis.OM.Modeling;

namespace Middleware.DataAccess.Dto;

[Document(IndexName = "robot-idx", StorageType = StorageType.Json, Prefixes = new[] { "Robot" })]
internal class RobotDto : Dto
{
    [Indexed]
    [RedisIdField]
    public string? Id { get; set; }
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


}
