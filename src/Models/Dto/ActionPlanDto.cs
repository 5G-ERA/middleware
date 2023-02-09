using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "actionPlan-idx", StorageType = StorageType.Json, Prefixes = new[] { "ActionPlan" })]
public class ActionPlanDto : Dto
{
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; }

    [Indexed]
    public string? TaskId { get; set; }
    [Indexed]
    public string Name { get; set; }
    [Indexed]
    public string? Status { get; set; }
    [Indexed]
    public bool IsReplan { get; set; }
    [Indexed(Sortable = true)]
    public DateTime LastStatusChange { get; set; }
    [Indexed]
    public List<ActionDto> ActionSequence { get; set; }
    [Indexed]
    public string RobotId { get; set; }
    [Indexed(Sortable = true)]
    public DateTime? TaskStartedAt { get; set; }
    
    public override BaseModel ToModel()
    {
        var dto = this;
        return new ActionPlanModel()
        {
            Id = Guid.Parse(dto.Id!),
            TaskId = Guid.Parse(dto.TaskId),
            Name = dto.Name,
            Status = dto.Status,
            IsReplan = dto.IsReplan,
            LastStatusChange = dto.LastStatusChange,
            ActionSequence = (List<ActionModel>)dto.ActionSequence.Select(x => x.ToModel()),
            RobotId = Guid.Parse(dto.RobotId),
            TaskStartedAt = (DateTime)dto.TaskStartedAt
        };
    }
}