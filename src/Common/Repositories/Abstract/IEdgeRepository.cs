using Middleware.Common.Models;

namespace Middleware.Common.Repositories.Abstract
{
    public interface IEdgeRepository : IBaseRepository<EdgeModel>
    {
        Task<EdgeModel> PatchEdgeAsync(Guid id, EdgeModel patch);
    }
}
