using Middleware.DataAccess.Dto.Ros;
using Redis.OM.Modeling;

namespace Middleware.DataAccess.Dto;

[Document(IndexName = "robot-idx", StorageType = StorageType.Json, Prefixes = new[] { "Robot" })]
internal class RobotDto
{
    [Indexed]
    public string? Id { get; set; }
    [Indexed]
    public string? Name { get; set; }
    [Indexed(JsonPath = "$.RosDistro")]
    public RosInfo Ros { get; set; } = new();
    [Indexed(Sortable = true)]
    public DateTimeOffset LastUpdatedTime { get; set; }
    [Indexed]
    public string? RobotModelName { get; set; }

    public string? RobotStatus { get; set; }
    public string Manufacturer { get; set; }

    public Uri ManufacturerUrl { get; set; }
    public long MaximumPayload { get; set; }

    public long MaximumTranslationalVelocity { get; set; }

    public long MaximumRotationalVelocity { get; set; }

    public long RobotWeight { get; set; }    

    public long BatteryStatus { get; set; }

    public string? MacAddress { get; set; }

    public string? LocomotionSystem { get; set; }

    public string? LocomotionType { get; set; } // Compulsory field

    public List<Sensor> Sensors { get; set; } = new();

    public List<Actuator> Actuators { get; set; } = new();

    public List<Manipulator> Manipulators { get; set; } = new();

    public List<string> QuestionIds { get; set; } = new();


}
