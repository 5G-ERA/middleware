using Microsoft.AspNetCore.Mvc;

namespace Middleware.RedisInterface.Contracts.Requests;

public class UpdateContainerRequest
{
    [FromRoute(Name = "id")]
    public Guid Id { get; init; }

    [FromBody]
    public ContainerRequest Container { get; set; } = default!;
}