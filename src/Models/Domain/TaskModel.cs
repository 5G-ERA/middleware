using System.Text.Json.Serialization;
using Middleware.Models.Dto;

namespace Middleware.Models.Domain;

public class TaskModel : BaseModel
{
    [JsonPropertyName("Id")]
    public override Guid Id { get; set; }

    [JsonPropertyName("Name")]
    public override string Name { get; set; } = default!;

    /// <summary>
    ///     //True: Dont change actions in action sequence replan
    ///     // True: The robot requests to not change anything from action sequence but placement.
    /// </summary>
    [JsonPropertyName("ReplanActionPlannerLocked")]
    public bool ReplanActionPlannerLocked { get; set; }

    /// <summary>
    ///     Should the resources be limited to the current deployment location
    ///     and should skip the reuse of the existing resources
    /// </summary>
    [JsonPropertyName("ResourceLock")]
    public bool ResourceLock { get; set; }

    [JsonPropertyName("TaskPriority")]
    public int TaskPriority { get; set; }

    /// <summary>
    ///     Action plan identifier assigned by the Middleware
    /// </summary>
    [JsonPropertyName("ActionPlanId")]
    public Guid ActionPlanId { get; set; }

    [JsonPropertyName("FullReplan")]
    public bool FullReplan { get; set; }

    [JsonPropertyName("PartialRePlan")]
    public bool PartialRePlan { get; set; }

    /// <summary>
    ///     No randomness in the task nature.
    /// </summary>
    [JsonPropertyName("DeterministicTask")]
    public bool DeterministicTask { get; set; }

    /// <summary>
    ///     Actions in action sequence are sequential or affect each other.
    /// </summary>
    [JsonPropertyName("MarkovianProcess")]
    public bool MarkovianProcess { get; set; }

    [JsonPropertyName("ActionSequence")]
    //[JsonIgnore]
    public List<ActionModel>? ActionSequence { get; set; }

    [JsonPropertyName("Tags")] //TODO: define allows tags
    //[JsonIgnore]
    public List<string>? Tags { get; set; }

    public TaskModel()
    {
    }

    public TaskModel(Guid id, int priority)
    {
        Id = id;
        TaskPriority = priority;
    }

    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new TaskDto
        {
            Id = domain.Id.ToString(),
            Name = domain.Name,
            TaskPriority = domain.TaskPriority,
            DeterministicTask = domain.DeterministicTask
        };
    }
}