using MassTransit;
using Middleware.Common;
using Middleware.Common.MessageContracts;

namespace Middleware.Orchestrator.Publishers;

internal class GatewayDeleteNetAppEntryPublisher : IPublisher<GatewayDeleteNetAppEntryMessage>
{
    private readonly IPublishEndpoint _publish;

    public GatewayDeleteNetAppEntryPublisher(IPublishEndpoint publish)
    {
        _publish = publish;
    }

    /// <inheritdoc />
    public async Task PublishAsync(GatewayDeleteNetAppEntryMessage message)
    {
        await _publish.Publish(message);
    }
}