using Middleware.Models.Dto;
using System.Text.Json.Serialization;

namespace Middleware.Models.Domain;

public sealed class HistoricalActionPlanModel : BaseModel
{
    [JsonPropertyName("Id")] //atuomatically generated plan id by middleware
    public override Guid Id { get; set; }

    [JsonPropertyName("TaskId")] //TaskID
    public Guid TaskId { get; set; }

    [JsonPropertyName("Name")] // Name of task
    public override string Name { get; set; }

    /// <summary>
    /// Status of the whole plan
    /// </summary>
    [JsonPropertyName("Status")]
    public string Status { get; set; }

    [JsonPropertyName("IsReplan")] //Status of whole plan
    public bool IsReplan { get; set; }

    [JsonPropertyName("PreviousPlanId")] //If replan, this field must be set to the id of the previous created HistoricalActionPlan.
    public Guid PreviousPlanId { get; set; }

    [JsonPropertyName("LastStatusChange")]
    public DateTime LastStatusChange { get; set; } // AL 2022-05-10: Not sure we need this one or how to use it.

    [JsonPropertyName("ActionSequence")]
    public List<ActionRunningModel> ActionSequence { get; set; }

    [JsonPropertyName("RobotId")]
    public Guid RobotId { get; set; }

    [JsonPropertyName("TaskStartedAt")]
    public DateTime TaskStartedAt { get; set; }

    [JsonPropertyName("CreationTime")]
    public DateTime CreationTime { get; set; }


    public void SetStatus(string status)
    {
        if (Status is null)
            TaskStartedAt = DateTime.UtcNow;

        Status = status;
        LastStatusChange = DateTime.UtcNow;
    }
    public override Dto.Dto ToDto()
    {
        var domain = this;
        return new HistoricalActionPlanDto()
        {
            Id = domain.Id.ToString(),
            TaskId = domain.TaskId.ToString(),
            Name = domain.Name,
            Status = domain.Status,
            IsReplan = domain.IsReplan,
            LastStatusChange = domain.LastStatusChange == default ? DateTimeOffset.Now : domain.LastStatusChange,
            ActionSequence = domain.ActionSequence,
            RobotId = domain.RobotId.ToString(),
            TaskStartedAt = domain.TaskStartedAt == default ? DateTimeOffset.Now : domain.TaskStartedAt,
            CreationTime = domain.CreationTime == default ? DateTimeOffset.Now : domain.TaskStartedAt,
            PreviousPlanId = domain.PreviousPlanId.ToString()
        }; ;
    }
}