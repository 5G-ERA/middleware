using MassTransit;
using Middleware.Common.MessageContracts;

namespace Middleware.TaskPlanner.Publishers;

public class SwitchoverDeployInstancePublisher : IPublisher<SwitchoverDeployAction>
{
    private readonly IPublishEndpoint _publish;

    public SwitchoverDeployInstancePublisher(IPublishEndpoint publish)
    {
        _publish = publish;
    }

    public async Task PublishAsync(SwitchoverDeployAction message)
    {
        await _publish.Publish(message);
    }
}