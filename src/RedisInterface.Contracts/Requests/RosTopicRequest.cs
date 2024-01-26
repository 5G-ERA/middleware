namespace Middleware.RedisInterface.Contracts.Requests;

public class RosTopicRequest
{
    public string Name { get; set; } = default!;


    public string Type { get; set; } = default!;

    public string Description { get; set; }
    public bool Enabled { get; set; }
    public string Compression { get; set; }

    public RosQosRequest Qos { get; set; }
}