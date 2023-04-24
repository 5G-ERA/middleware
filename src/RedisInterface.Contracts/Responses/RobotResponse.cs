using Middleware.Models.Domain;

namespace Middleware.RedisInterface.Contracts.Responses;

public class RobotResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; }
    public string? ModelName { get; init; }
    public string? Status { get; init; }
    public int BatteryStatus { get; init; }
    public int RosVersion { get; init; }
    public string RosDistro { get; init; }
    public long? MaximumPayload { get; init; }
    public long? MaximumTranslationalVelocity { get; init; }
    public long? MaximumRotationalVelocity { get; init; }
    public long? RobotWeight { get; init; }
    public Uri? RosRepo { get; init; }
    public IEnumerable<ROSNodeModel>? RosNodes { get; init; }
    public string? Manufacturer { get; init; }
    public Uri? ManufacturerUrl { get; init; }
    public string? MacAddress { get; init; }
    public string? LocomotionSystem { get; init; }
    public string? LocomotionType { get; init; }
    public IEnumerable<SensorModel>? Sensors { get; init; }
    public IEnumerable<ActuatorModel>? Actuators { get; init; }
    public IEnumerable<ManipulatorModel>? Manipulators { get; init; }
    public int? Cpu { get; init; }
    public int? NumberOfCores { get; init; }
    public long? Ram { get; init; }
    public long? StorageDisk { get; init; }
    public IEnumerable<DialogueModel>? Questions { get; init; }
    public DateTime LastUpdatedTime { get; init; }
    public DateTime OnboardedTime { get; init; }
}