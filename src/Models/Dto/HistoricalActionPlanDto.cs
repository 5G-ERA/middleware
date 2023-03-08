using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "historicalActionPlan-idx", StorageType = StorageType.Json, Prefixes = new[] { HistoricalActionPlanDto.Prefix })]
public class HistoricalActionPlanDto : Dto
{
    public const string Prefix = "HistoricalActionPlan";
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
    public DateTimeOffset LastStatusChange { get; set; }
    [Indexed]
    public List<ActionRunningModel> ActionSequence { get; set; }
    [Indexed]
    public string RobotId { get; set; }
    [Indexed(Sortable = true)]
    public DateTimeOffset TaskStartedAt { get; set; }
    [Indexed]
    public string PreviousPlanId { get; set; }

public override BaseModel ToModel()
    {
        var dto = this;
        return new HistoricalActionPlanModel()
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            TaskId = Guid.Parse(dto.TaskId),
            Name = dto.Name,
            Status = dto.Status,
            IsReplan = dto.IsReplan,
            LastStatusChange = dto.LastStatusChange.DateTime,
            ActionSequence = dto.ActionSequence,
            RobotId = Guid.Parse(dto.RobotId),
            TaskStartedAt = dto.TaskStartedAt.DateTime,
            PreviousPlanId = Guid.Parse(dto.PreviousPlanId)
        };
    }
}