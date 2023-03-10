using Microsoft.AspNetCore.Mvc;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Requests;

public class UpdateRobotRequest
{
    [FromRoute(Name = "id")]
    public Guid Id { get; init; }

    [FromBody]
    public RobotRequest Robot { get; set; } = default!;
}