namespace Middleware.RedisInterface.Contracts.Requests;

public class RosActionRequest
{
    public string Name { get; set; } = default!;
    public string Type { get; set; } = default!;
}