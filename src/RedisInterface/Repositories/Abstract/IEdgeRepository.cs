using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories.Abstract
{
    public interface IEdgeRepository : IBaseRepository<EdgeModel>
    {
        Task<EdgeModel> PatchEdgeAsync(Guid id, EdgeModel patch);
    }
}
