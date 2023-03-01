using Microsoft.AspNetCore.Mvc;

namespace Middleware.RedisInterface.Contracts.Requests;

public class UpdateRobotRequest
{
    [FromRoute(Name = "id")]
    public Guid Id { get; init; }

    [FromBody]
    public RobotRequest Robot { get; set; } = default!;
}