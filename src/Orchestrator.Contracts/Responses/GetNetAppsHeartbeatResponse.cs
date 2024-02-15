using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Contracts.Responses;

public class GetNetAppsHeartbeatResponse
{
    public IEnumerable<GetNetAppHeartbeatResponse> NetApps { get; set; } = Enumerable.Empty<GetNetAppHeartbeatResponse>();
}