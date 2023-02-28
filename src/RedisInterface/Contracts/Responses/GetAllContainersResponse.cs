namespace Middleware.RedisInterface.Contracts.Responses;

public class GetAllContainersResponse
{
    public IEnumerable<ContainerResponse> Containers { get; set; } = Enumerable.Empty<ContainerResponse>();
}