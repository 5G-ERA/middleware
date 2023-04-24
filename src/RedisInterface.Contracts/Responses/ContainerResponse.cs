namespace Middleware.RedisInterface.Contracts.Responses;

public class ContainerResponse
{
    public Guid Id { get; init; }
    public string Name { get; init; }
    public DateTime LastUpdateTime { get; init; }
    public string? Description { get; init; } = default!;
    public string K8sDeployment  { get; init; }
    public string? K8sService { get; init; } = default!;
}