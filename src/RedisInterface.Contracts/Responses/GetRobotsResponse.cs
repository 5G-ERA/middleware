namespace Middleware.RedisInterface.Contracts.Responses;

public class GetRobotsResponse
{
    public IEnumerable<RobotResponse> Robots { get; init; } = Enumerable.Empty<RobotResponse>();
}