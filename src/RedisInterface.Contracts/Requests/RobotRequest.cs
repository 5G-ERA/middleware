using Middleware.Models.Domain;
using Middleware.Models.Domain.Ros;

namespace Middleware.RedisInterface.Contracts.Requests;

public class RobotRequest
{
    public string Name { get; init; }
    public string ModelName { get; init; }
    public string Status { get; init; }
    public int BatteryStatus { get; init; }
    public int RosVersion { get; init; }
    public string RosDistro { get; init; }
    public long? MaximumPayload { get; init; }
    public long? MaximumTranslationalVelocity { get; init; }
    public long? MaximumRotationalVelocity { get; init; }
    public long? RobotWeight { get; init; }
    public Uri RosRepo { get; init; }
    public IEnumerable<RosNodeModel> RosNodes { get; init; }
    public string Manufacturer { get; init; }
    public Uri ManufacturerUrl { get; init; }
    public string MacAddress { get; init; }
    public string LocomotionSystem { get; init; }
    public string LocomotionType { get; init; }
    public IEnumerable<SensorModel> Sensors { get; init; }
    public IEnumerable<ActuatorModel> Actuators { get; init; }
    public IEnumerable<ManipulatorModel> Manipulators { get; init; }
    public int? Cpu { get; init; }
    public int? NumberOfCores { get; init; }
    public long? Ram { get; init; }
    public long? StorageDisk { get; init; }
    public IEnumerable<DialogueModel> Questions { get; init; }
    public string SimCardNumber { get; init; }
}