namespace Middleware.RedisInterface.Contracts.Requests;

public class ActionRequest
{
    public string Name { get; init; } = default!;
    public string Priority { get; init; } = default!;
    public int Order { get; init; }
    public int? MinimumNumCores { get; init; }
    public long? MinimumRam { get; init; }
    public IEnumerable<string> Tags { get; init; } = Enumerable.Empty<string>();

}