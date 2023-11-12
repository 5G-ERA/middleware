using System.Collections.Immutable;
using Middleware.Models.Domain;
using Middleware.Models.Dto;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IEdgeRepository : IRedisRepository<EdgeModel, EdgeDto>, IRelationRepository
    {
        Task<EdgeModel> PatchEdgeAsync(Guid id, EdgeModel patch);
        Task<List<EdgeModel>> GetFreeEdgesIdsAsync(List<EdgeModel> listofEdgesConnectedtoRobot);
        Task<List<EdgeModel>> GetLessBusyEdgesAsync(List<EdgeModel> busyEdgesTocheck);

        Task<EdgeModel?> GetEdgeResourceDetailsByNameAsync(string name);

        Task<bool> IsBusyEdgeByIdAsync(Guid edgeId);

        Task<bool> IsBusyEdgeByNameAsync(string edgeName);

        Task<int> GetNumContainersByIdAsync(Guid edgeId);

        Task<int> GetNumContainersByNameAsync(string edgeName);

        Task<ImmutableList<EdgeModel>> GetEdgesByOrganizationAsync(string organization);

        Task<bool> CheckIfAddressExists(Uri address);
        Task<(bool, EdgeModel?)> CheckIfNameExists(string name);

        Task<(bool, EdgeModel? matchedEdge)> CheckIfIdExists(string id);

        Task<CloudEdgeStatusResponse> GetEdgeOnlineStatusLastUpdatedTimeAsync(Guid cloudId);

        Task SetEdgeOnlineStatusAsync(Guid cloudId, bool isOnline);

    }
}
