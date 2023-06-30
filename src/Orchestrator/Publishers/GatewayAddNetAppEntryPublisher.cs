using MassTransit;
using Middleware.Common;
using Middleware.Common.MessageContracts;

namespace Middleware.Orchestrator.Publishers;

internal class GatewayAddNetAppEntryPublisher : IPublisher<GatewayAddNetAppEntryMessage>
{
    private readonly IPublishEndpoint _publish;

    public GatewayAddNetAppEntryPublisher(IPublishEndpoint publish)
    {
        _publish = publish;
    }

    /// <inheritdoc />
    public async Task PublishAsync(GatewayAddNetAppEntryMessage message)
    {
        await _publish.Publish(message);
    }
}