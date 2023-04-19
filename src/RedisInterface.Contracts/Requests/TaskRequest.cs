namespace Middleware.RedisInterface.Contracts.Requests;

public class TaskRequest
{
    public string Name { get; init; }
    public string Priority { get; init; }
    public bool IsDeterministic { get; init; }
    public IEnumerable<string> Tags { get; init; }
}