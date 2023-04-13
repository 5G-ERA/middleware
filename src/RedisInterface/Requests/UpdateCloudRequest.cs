using Microsoft.AspNetCore.Mvc;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Requests;

public class UpdateCloudRequest
{
    [FromRoute(Name = "id")]
    public Guid Id { get; init; }

    [FromBody]
    public CloudRequest Cloud { get; set; } = default!;
}