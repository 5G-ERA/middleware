using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IInstanceRepository : IBaseRepository<InstanceModel>, IRelationRepository
    {
        Task<InstanceModel> PatchInstanceAsync(Guid id, InstanceModel patch);

        Task<InstanceModel?> FindAlternativeInstance(Guid instance);
    }
}
