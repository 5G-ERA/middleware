using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "task-idx", StorageType = StorageType.Json, Prefixes = new[] { TaskDto.Prefix })]
public class TaskDto : Dto
{
    public const string Prefix = "Task";
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; }

    [Indexed]
    public string? Name { get; set; }
    [Indexed]
    public bool ReplanActionPlannerLocked { get; set; }
    [Indexed]
    public bool ResourceLock { get; set; }
    [Indexed(Sortable = true)]
    public int TaskPriority { get; set; }
    [Indexed]
    public string ActionPlanId { get; set; }
    [Indexed]
    public bool FullReplan { get; set; }
    [Indexed]
    public bool PartialRePlan { get; set; }
    [Indexed(Sortable = true)]
    public bool DeterministicTask { get; set; } // The result is always the same if true.
    [Indexed]
    public bool MarkovianProcess { get; set; } // If true, actions are not independant and need to be executed in order.
    [Indexed]
    public List<ActionDto> ActionSequence { get; set; }
    [Indexed]
    public List<string> Tags { get; set; }
 

    public override BaseModel ToModel()
    {
        var dto = this;
        return new TaskModel()
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            Name = dto.Name,
            ReplanActionPlannerLocked = dto.ReplanActionPlannerLocked,
            ResourceLock = dto.ResourceLock,
            TaskPriority = dto.TaskPriority,
            ActionPlanId = Guid.Parse(dto.ActionPlanId),
            FullReplan = dto.FullReplan,
            PartialRePlan = dto.PartialRePlan,
            DeterministicTask = dto.DeterministicTask,
            MarkovianProcess = dto.MarkovianProcess,
            ActionSequence = (List<ActionModel>)dto.ActionSequence.Select(x => x.ToModel()),
            Tags = dto.Tags
        };
    }
}