namespace Middleware.RedisInterface.Contracts.Responses;

public class GetContainersResponse
{
    public IEnumerable<ContainerResponse> Containers { get; set; } = Enumerable.Empty<ContainerResponse>();
}