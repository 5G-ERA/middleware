using MassTransit;
using Middleware.Common;
using Middleware.Common.MessageContracts;

namespace Middleware.TaskPlanner.Publishers;

internal class
    ResourcePlanRequestPublisher : IRequestResponseClient<RequestResourcePlanMessage, RequestResourcePlanMessage>
{
    private readonly IRequestClient<RequestResourcePlanMessage> _requestClient;

    public ResourcePlanRequestPublisher(IRequestClient<RequestResourcePlanMessage> requestClient)
    {
        _requestClient = requestClient;
    }

    /// <inheritdoc />
    public async Task<RequestResourcePlanMessage> Request(RequestResourcePlanMessage request)
    {
        var resp = await _requestClient.GetResponse<RequestResourcePlanMessage>(request);
        return resp.Message;
    }
}