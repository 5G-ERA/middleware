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
        try
        {
            var resourcePlan = await _resourcePlanner.Plan(context.Message.Task, context.Message.Robot);

            var resp = new RequestResourcePlanMessage
            {
                Task = resourcePlan,
                IsSuccess = true
            };
            await context.RespondAsync(resp);
        }
        catch (Exception ex)
        {
            var resp = new RequestResourcePlanMessage
            {
                Task = null,
                IsSuccess = false,
                Error = ex.ToString()
            };
            await context.RespondAsync(resp);
        }
    }
}