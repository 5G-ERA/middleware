using Microsoft.AspNetCore.JsonPatch;
using Middleware.Common.Models;

namespace DataAccess.Repositories.Abstract
{
    public interface IInstanceRepository : IBaseRepository<InstanceModel>
    {
        Task<InstanceModel> PatchInstanceAsync(Guid id, InstanceModel patch);

        Task<InstanceModel> FindAlternativeInstance(Guid instance);
    }
}
