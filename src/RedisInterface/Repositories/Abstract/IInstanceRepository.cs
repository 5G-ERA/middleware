using Microsoft.AspNetCore.JsonPatch;
using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories
{
    public interface IInstanceRepository : IBaseRepository<InstanceModel>
    {
        Task<InstanceModel> PatchInstanceAsync(Guid id, InstanceModel patch);
    }
}
