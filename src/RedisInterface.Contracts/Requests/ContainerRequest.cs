namespace Middleware.RedisInterface.Contracts.Requests;

public class ContainerRequest
{
    public string Name { get; init; }
    public string Description { get; init; } = default!;
    public string K8SDeployment { get; init; } = default!;
    public string K8SService { get; init; } = default!;
}