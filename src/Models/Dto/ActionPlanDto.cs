using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "actionPlan-idx", StorageType = StorageType.Json, Prefixes = new[] { ActionPlanDto.Prefix })]
public class ActionPlanDto : Dto
{
    public const string Prefix = "ActionPlan";

    [Indexed]
    [RedisIdField]
    public override string? Id { get; set; }

    [Indexed]
    public string TaskId { get; set; }

    [Indexed]
    public string? Name { get; set; }

    [Indexed]
    public string Status { get; set; }

    [Indexed]
    public bool IsReplan { get; set; }

    [Indexed(Sortable = true)]
    public DateTimeOffset LastStatusChange { get; set; }

    [Indexed]
    public List<ActionModel>? ActionSequence { get; set; }

    [Indexed]
    public string RobotId { get; set; }

    [Indexed(Sortable = true)]
    public DateTimeOffset TaskStartedAt { get; set; }
    
    public override BaseModel ToModel()
    {
        var dto = this;
        return new ActionPlanModel()
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            TaskId = Guid.Parse(dto.TaskId),
            Name = dto.Name,
            Status = dto.Status,
            IsReplan = dto.IsReplan,
            LastStatusChange = dto.LastStatusChange.DateTime,
            ActionSequence = (List<ActionModel>)dto.ActionSequence,
            RobotId = Guid.Parse(dto.RobotId),
            TaskStartedAt = dto.TaskStartedAt.DateTime
        };
    }
}