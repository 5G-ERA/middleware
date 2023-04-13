using Microsoft.AspNetCore.Mvc;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Requests;

public class UpdateEdgeRequest
{
    [FromRoute(Name = "id")]
    public Guid Id { get; init; }

    [FromBody]
    public EdgeRequest Edge { get; set; } = default!;
}