namespace Middleware.RedisInterface.Contracts.Requests;

public class ActionPlanRequest
{
    public Guid Id { get; set; }

    public Guid TaskId { get; set; }

    public string Name { get; set; } = default!;

    public string Status { get; set; }

    public bool IsReplan { get; set; }

    public IEnumerable<RunningActionRequest> ActionSequence { get; set; } = Enumerable.Empty<RunningActionRequest>();

    public Guid RobotId { get; set; }

    public DateTime TaskStartedAt { get; set; }
}