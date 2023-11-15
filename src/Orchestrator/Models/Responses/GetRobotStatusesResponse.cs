using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Models.Responses;

public class GetRobotStatusesResponse
{
    public IEnumerable<RobotStatusModel> Robots { get; set; } = Enumerable.Empty<RobotStatusModel>();
}