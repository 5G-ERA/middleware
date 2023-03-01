namespace Middleware.RedisInterface.Contracts.Responses;

public class GetAllTasksResponse
{
    public IEnumerable<TaskResponse> Tasks { get; set; } = Enumerable.Empty<TaskResponse>();
}