namespace Middleware.RedisInterface.Contracts.Responses;

public class GetRunningActionsResponse
{
    public IEnumerable<RunningActionResponse> RunningActions { get; set; } = Enumerable.Empty<RunningActionResponse>();
}