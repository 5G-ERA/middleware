using Middleware.Models.Dto;

namespace Middleware.Models.Domain;

public sealed class ActionPlanModel : BaseModel
{
    /// <summary>
    ///     Automatically generated plan id by middleware
    /// </summary>
    public override Guid Id { get; set; }

    /// <summary>
    ///     Identifier of the Task that is executed
    /// </summary>
    public Guid TaskId { get; set; }

    /// <summary>
    ///     Name of the executed task
    /// </summary>
    public override string Name { get; set; } = null!;

    /// <summary>
    ///     Status of the whole plan
    /// </summary>

    public string? Status { get; set; }

    /// <summary>
    ///     Status of whole plan
    /// </summary>
    public bool IsReplan { get; set; }

    /// <summary>
    ///     Last time the status of the Task has been changed
    ///     AL 2022-05-10: Not sure we need this one or how to use it.
    /// </summary>
    public DateTime LastStatusChange { get; set; }

    public List<ActionModel>? ActionSequence { get; set; }


    public Guid RobotId { get; set; }

    public DateTime TaskStartedAt { get; set; }

    public ActionPlanModel()
    {
    }

    public ActionPlanModel(Guid id, Guid taskId, string name, List<ActionModel>? actionSequence, Guid robotId)
    {
        Id = id;
        TaskId = taskId;
        Name = name;
        ActionSequence = actionSequence;
        RobotId = robotId;
    }

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
        return new ActionPlanDto
        {
            Id = domain.Id.ToString(),
            TaskId = domain.TaskId.ToString(),
            Name = domain.Name,
            Status = domain.Status,
            IsReplan = domain.IsReplan,
            LastStatusChange = domain.LastStatusChange == default ? DateTimeOffset.Now : domain.LastStatusChange,
            ActionSequence = domain.ActionSequence,
            RobotId = domain.RobotId.ToString(),
            TaskStartedAt = domain.TaskStartedAt == default ? DateTimeOffset.Now : domain.TaskStartedAt
        };
        ;
    }
}