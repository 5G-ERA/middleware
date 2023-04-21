using Microsoft.AspNetCore.Mvc;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Requests;

public class UpdateContainerRequest
{
    [FromRoute(Name = "id")]
    public Guid Id { get; init; }

    [FromBody]
    public ContainerRequest Container { get; set; } = default!;
}