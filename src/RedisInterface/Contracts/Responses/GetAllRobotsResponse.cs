namespace Middleware.RedisInterface.Contracts.Responses;

public class GetAllRobotsResponse
{
    public IEnumerable<RobotResponse> Robots { get; init; } = Enumerable.Empty<RobotResponse>();
}