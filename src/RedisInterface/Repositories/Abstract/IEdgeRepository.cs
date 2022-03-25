using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories.Abstract
{
    public interface IEdgeRepository : IBaseRepository<EdgeModel>
    {
        Task<List<RelationModel>> GetRelation(Guid id, string relationName);
    }
}
