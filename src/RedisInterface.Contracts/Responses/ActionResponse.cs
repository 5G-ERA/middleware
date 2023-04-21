namespace Middleware.RedisInterface.Contracts.Responses;

public class ActionResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; } = default!;
    
    public string Priority { get; init; } = default!;
    public int Order { get; init; }
    public int? MinimumNumCores { get; init; }
    public long? MinimumRam { get; init; }
    public IEnumerable<string> Tags { get; init; } = Enumerable.Empty<string>();

}