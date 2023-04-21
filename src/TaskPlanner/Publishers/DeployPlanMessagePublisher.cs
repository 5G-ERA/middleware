using MassTransit;
using Middleware.Common.MessageContracts;

namespace Middleware.TaskPlanner.Publishers;

public class DeployPlanMessagePublisher  : IPublisher<DeployPlanMessage>
{
    private readonly IPublishEndpoint _publish;

    public DeployPlanMessagePublisher(IPublishEndpoint publish)
    {
        _publish = publish;
    }
    
    public async Task PublishAsync(DeployPlanMessage message)
    {
        await _publish.Publish(message);
    }
}