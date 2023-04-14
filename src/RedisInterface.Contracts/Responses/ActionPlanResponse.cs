namespace Middleware.RedisInterface.Contracts.Responses;

public class ActionPlanResponse
{
    public Guid Id { get; set; }

    public Guid TaskId { get; set; }

    public string Name { get; set; } = default!;

    public string Status { get; set; }

    public bool IsReplan { get; set; }

    public IEnumerable<RunningActionResponse> ActionSequence { get; set; } = Enumerable.Empty<RunningActionResponse>();

    public Guid RobotId { get; set; }

    public DateTime TaskStartedAt { get; set; }
}