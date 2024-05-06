namespace Middleware.Orchestrator.Contracts.Responses;

public class GetRobotsHeartbeatResponse
{
    public IEnumerable<GetRobotHeartbeatResponse> Robots { get; set; } = Enumerable.Empty<GetRobotHeartbeatResponse>();
}