using Middleware.Common.Models;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IInstanceRepository : IBaseRepository<InstanceModel>
    {
        Task<InstanceModel> PatchInstanceAsync(Guid id, InstanceModel patch);

        Task<InstanceModel> FindAlternativeInstance(Guid instance);
    }
}
