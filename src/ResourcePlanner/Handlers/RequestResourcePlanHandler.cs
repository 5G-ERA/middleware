using MassTransit;
using Middleware.Common.MessageContracts;

namespace Middleware.ResourcePlanner.Handlers;

internal class RequestResourcePlanHandler : IConsumer<RequestResourcePlanMessage>
{
    private readonly IResourcePlanner _resourcePlanner;

    public RequestResourcePlanHandler(IResourcePlanner resourcePlanner)
    {
        _resourcePlanner = resourcePlanner;
    }

    /// <inheritdoc />
    public async Task Consume(ConsumeContext<RequestResourcePlanMessage> context)
    {
        var resourcePlan = await _resourcePlanner.Plan(context.Message.Task, context.Message.Robot);

        var resp = new RequestResourcePlanMessage
        {
            Task = resourcePlan
        };
        await context.RespondAsync(resp);
    }
}