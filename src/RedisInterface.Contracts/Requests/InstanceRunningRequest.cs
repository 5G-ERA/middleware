namespace Middleware.RedisInterface.Contracts.Requests;

public class InstanceRunningRequest
{
    public Guid Id { get; set; }

    public string Name { get; set; }

    public Guid ServiceInstanceId { get; set; }

    public string ServiceType { get; set; }

    public string ServiceUrl { get; set; }

    public string ServiceStatus { get; set; }

    public DateTime DeployedTime { get; set; }
}