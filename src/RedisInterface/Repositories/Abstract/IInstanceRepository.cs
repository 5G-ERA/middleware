using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories
{
    public interface IInstanceRepository
    {
        Task<InstanceModel> GetInstanceByIdAsync(Guid id);

        Task<InstanceModel> PostInstanceAsync();

        Task<InstanceModel> PatchInstanceAsync();

        Task DeleteInstance(Guid id);


    }
}
