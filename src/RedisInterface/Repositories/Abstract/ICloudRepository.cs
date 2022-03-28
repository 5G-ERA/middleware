using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories.Abstract
{
    public interface ICloudRepository : IBaseRepository<CloudModel>
    {

        Task<List<RelationModel>> GetRelation(Guid id, string relationName);

        Task<CloudModel> PatchCloudAsync(Guid id, CloudModel patch);

    }
}
