using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Models.Responses;

public class GetNetAppStatusesResponse
{
    public IEnumerable<NetAppStatusModel> NetApps { get; set; } = Enumerable.Empty<NetAppStatusModel>();
}