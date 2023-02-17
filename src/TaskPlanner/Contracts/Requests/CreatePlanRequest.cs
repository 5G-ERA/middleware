using System.Text.Json.Serialization;
using Middleware.Common.Models;

namespace Middleware.TaskPlanner.Contracts.Requests;

public class CreatePlanRequest
{
    [JsonPropertyName("RobotId")]
    public Guid RobotId { get; init; }

    /// <summary>
    /// Lock the resource planning only to use resources from the currently connected middleware 
    /// </summary>
    [JsonPropertyName("LockResourceReUse")]
    public bool LockResourceReUse { get; init; } = true;

    [JsonPropertyName("TaskId")]
    public Guid Id { get; init; }

    /// <summary>
    /// Use the hardcoded action plan if true, use the semantic planning if false
    /// </summary>        
    [JsonPropertyName("ContextKnown")]
    public bool ContextKnown { get; init; } = true; 

    [JsonPropertyName("Questions")]
    public List<DialogueModel> Questions { get; init; }

    public bool IsValid()
    {
        return RobotId != Guid.Empty && Id != Guid.Empty;
    }
}