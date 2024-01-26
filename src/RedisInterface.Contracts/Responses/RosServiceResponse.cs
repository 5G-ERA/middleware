namespace Middleware.RedisInterface.Contracts.Responses;

public class RosServiceResponse
{
    public string Name { get; init; } = default!;
    public string Type { get; init; } = default!;
    public string Description { get; init; }

    public RosQosResponse Qos { get; init; }
}