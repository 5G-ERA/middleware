using MassTransit;
using Middleware.Common.MessageContracts;

namespace Middleware.Orchestrator.Handlers.Switchover;

public class SwitchoverDeleteInstanceConsumer : IConsumer<SwitchoverDeleteInstance>
{
    public async Task Consume(ConsumeContext<SwitchoverDeleteInstance> context)
    {
        throw new NotImplementedException();
    }
}