namespace Middleware.RedisInterface.Contracts.Responses;

public class GetActionPlansResponse
{
    public IEnumerable<ActionPlanResponse> ActionPlans { get; set; } = Enumerable.Empty<ActionPlanResponse>();
}