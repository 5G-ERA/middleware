namespace Middleware.RedisInterface.Contracts.Responses;

public class GetTasksResponse
{
    public IEnumerable<TaskResponse> Tasks { get; set; } = Enumerable.Empty<TaskResponse>();
}