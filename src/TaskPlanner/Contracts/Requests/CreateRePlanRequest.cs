using System.Text.Json.Serialization;
using Middleware.Common.Models;

namespace Middleware.TaskPlanner.Contracts.Requests;

public class CreateRePlanRequest 
{
    [JsonPropertyName("RobotId")]
    public Guid RobotId { get; set; }

    [JsonPropertyName("LockResourceReUse")]
    public bool LockResourceReUse { get; set; }

    [JsonPropertyName("ContextKnown")]
    public bool ContextKnown { get; set; }

    [JsonPropertyName("CompleteReplan")]
    public bool CompleteReplan { get; set; }

    [JsonPropertyName("TaskID")]
    public Guid TaskID { get; set; }

    [JsonPropertyName("Questions")]
    public List<DialogueModel> Questions { get; set; }

}