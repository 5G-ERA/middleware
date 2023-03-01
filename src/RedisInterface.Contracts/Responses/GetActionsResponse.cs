namespace Middleware.RedisInterface.Contracts.Responses;

public class GetActionsResponse
{
    public IEnumerable<ActionResponse> Actions { get; init; } = Enumerable.Empty<ActionResponse>();
}