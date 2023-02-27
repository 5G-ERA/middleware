namespace Middleware.RedisInterface.Contracts.Responses;

public class GetAllActionsResponse
{
    public IEnumerable<ActionResponse> Actions { get; init; } = Enumerable.Empty<ActionResponse>();
}