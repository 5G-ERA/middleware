using Microsoft.AspNetCore.Mvc;

namespace Middleware.RedisInterface.Contracts.Requests;

public class UpdateTaskRequest
{
    [FromRoute(Name = "id")]
    public Guid Id { get; init; }

    [FromBody]
    public TaskRequest Task { get; set; } = default!;
}