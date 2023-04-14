using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IInstanceRunningRepository : IBaseRepository<InstanceRunningModel>, IRelationRepository
    {
        Task<InstanceRunningModel> PatchInstanceAsync(Guid id, InstanceRunningModel patch);

    }
}
