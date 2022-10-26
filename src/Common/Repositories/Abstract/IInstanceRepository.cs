using Microsoft.AspNetCore.JsonPatch;
using Middleware.Common.Models;

namespace Middleware.Common.Repositories
{
    public interface IInstanceRepository : IBaseRepository<InstanceModel>
    {
        Task<InstanceModel> PatchInstanceAsync(Guid id, InstanceModel patch);

        Task<InstanceModel> FindAlternativeInstance(Guid instance);
    }
}
