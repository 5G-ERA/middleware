using MassTransit;
using Middleware.Common;
using Middleware.Common.MessageContracts;

namespace Middleware.TaskPlanner.Publishers;

internal class ConnectRobotToSlicePublisher : IPublisher<ConnectRobotToSliceMessage>
{
    private readonly IPublishEndpoint _publish;

    public ConnectRobotToSlicePublisher(IPublishEndpoint publish)
    {
        _publish = publish;
    }

    public async Task PublishAsync(ConnectRobotToSliceMessage message)
    {
        await _publish.Publish(message);
    }
}