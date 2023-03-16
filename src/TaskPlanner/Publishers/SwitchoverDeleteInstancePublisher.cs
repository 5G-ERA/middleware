using MassTransit;
using Middleware.Common.MessageContracts;

namespace Middleware.TaskPlanner.Publishers;

public class SwitchoverDeleteInstancePublisher : IPublisher<SwitchoverDeleteAction>
{
    private readonly IPublishEndpoint _publish;

    public SwitchoverDeleteInstancePublisher(IPublishEndpoint publish)
    {
        _publish = publish;
    }

    public async Task PublishAsync(SwitchoverDeleteAction message)
    {
        await _publish.Publish(message);
    }
}