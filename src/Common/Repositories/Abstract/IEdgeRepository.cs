using Middleware.Common.Models;

namespace Middleware.Common.Repositories.Abstract
{
    public interface IEdgeRepository : IBaseRepository<EdgeModel>
    {
        Task<EdgeModel> PatchEdgeAsync(Guid id, EdgeModel patch);
        Task<List<EdgeModel>> GetFreeEdgesIdsAsync(List<EdgeModel> listofEdgesConnectedtoRobot);
        Task<List<EdgeModel>> GetLessBusyEdgesAsync(List<EdgeModel> busyEdgesTocheck);

        Task<List<EdgeModel>> GetEdgeResourceDetailsbyNameAsync(string name);
    }
}
