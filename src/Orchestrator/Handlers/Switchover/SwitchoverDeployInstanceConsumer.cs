using MassTransit;
using Middleware.Common.MessageContracts;

namespace Middleware.Orchestrator.Handlers.Switchover;

public class SwitchoverDeployInstanceConsumer : IConsumer<SwitchoverDeployInstance>
{
    public async Task Consume(ConsumeContext<SwitchoverDeployInstance> context)
    {
        throw new NotImplementedException();
    }
}