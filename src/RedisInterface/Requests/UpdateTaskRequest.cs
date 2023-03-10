using Microsoft.AspNetCore.Mvc;
using Middleware.RedisInterface.Contracts.Requests;

namespace Middleware.RedisInterface.Requests;

public class UpdateTaskRequest
{
    [FromRoute(Name = "id")]
    public Guid Id { get; init; }

    [FromBody]
    public TaskRequest Task { get; set; } = default!;
}