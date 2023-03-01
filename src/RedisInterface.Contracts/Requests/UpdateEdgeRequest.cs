using Microsoft.AspNetCore.Mvc;

namespace Middleware.RedisInterface.Contracts.Requests;

public class UpdateEdgeRequest
{
    [FromRoute(Name = "id")]
    public Guid Id { get; init; }

    [FromBody]
    public EdgeRequest Edge { get; set; } = default!;
}