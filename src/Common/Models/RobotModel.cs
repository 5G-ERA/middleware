using System.Text.Json.Serialization;

namespace Middleware.Common.Models;

public class RobotModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("RobotName")]
    public override string Name { get; set; }

    [JsonPropertyName("Manufacturer")]
    public string Manufacturer { get; set; }

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

    [JsonPropertyName("LocomotionSystem")]
    public string LocomotionSystem { get; set; }

    [JsonPropertyName("Sensors")]
    public List<string> Sensors { get; set; }

    [JsonPropertyName("CPU")]
    public long Cpu { get; set; }

    [JsonPropertyName("RAM")]
    public long Ram { get; set; }

    [JsonPropertyName("VirtualRam")]
    public long VirtualRam { get; set; }

    [JsonPropertyName("StorageDisk")]
    public long StorageDisk { get; set; }

    [JsonPropertyName("NumberCores")]
    public long NumberCores { get; set; }

    [JsonPropertyName("Questions")]
    public List<DialogueModel> Questions { get; set; }
}
