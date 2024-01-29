namespace Middleware.RedisInterface.Contracts.Requests;

public class RosServiceRequest
{
    public string Name { get; init; } = default!;
    public string Type { get; init; } = default!;
    public string Description { get; init; }

    public RosQosRequest Qos { get; init; }
}