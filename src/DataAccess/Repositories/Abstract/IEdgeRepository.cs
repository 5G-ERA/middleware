using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IEdgeRepository : IBaseRepository<EdgeModel>, IRelationRepository
    {
        Task<EdgeModel> PatchEdgeAsync(Guid id, EdgeModel patch);
        Task<List<EdgeModel>> GetFreeEdgesIdsAsync(List<EdgeModel> listofEdgesConnectedtoRobot);
        Task<List<EdgeModel>> GetLessBusyEdgesAsync(List<EdgeModel> busyEdgesTocheck);

        Task<EdgeModel> GetEdgeResourceDetailsByNameAsync(string name);

        Task<bool> IsBusyEdgeByIdAsync(Guid edgeId);

        Task<bool> IsBusyEdgeByNameAsync(string edgeName);

        Task<int> GetNumContainersByIdAsync(Guid edgeId);

        Task<int> GetNumContainersByNameAsync(string edgeName);
    }
}
