using MassTransit;
using Middleware.Common.MessageContracts;
using Middleware.Orchestrator.SliceManager.Contracts;

namespace Middleware.Orchestrator.Handlers.Slice;

internal class ConnectRobotToSliceConsumer : IConsumer<ConnectRobotToSliceMessage>
{
    private readonly ILogger<ConnectRobotToSliceConsumer> _logger;
    private readonly ISliceManagerClientFactory _sliceManagerClientFactory;

    public ConnectRobotToSliceConsumer(ISliceManagerClientFactory sliceManagerClientFactory,
        ILogger<ConnectRobotToSliceConsumer> logger)
    {
        _sliceManagerClientFactory = sliceManagerClientFactory;
        _logger = logger;
    }

    public async Task Consume(ConsumeContext<ConnectRobotToSliceMessage> context)
    {
        _logger.LogDebug("Started processing ConnectRobotToSliceMessage");

        var message = context.Message;
        if (_sliceManagerClientFactory.IsSlicingAvailable() == false)
        {
            _logger.LogError(
                "Requested Slice association between Robot and Slice in the location without Slicing mechanism configured! ActionPlanId: {0}",
                message.ActionPlanId);
            return;
        }

        var slice = _sliceManagerClientFactory.CreateSliceManagerClient();
        //TODO: select correct values for up link and down link
        await slice.AttachImsiToSlice(message.Imsi, message.Slice, 100, 100);
    }
}