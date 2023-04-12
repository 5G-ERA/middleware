namespace Middleware.RedisInterface.Contracts.Responses;

public class TaskResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; }
    public string Priority { get; init; }
    public bool IsDeterministic { get; init; }
    public IEnumerable<string> Tags { get; init; } = new List<string>();
}