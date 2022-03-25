using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories.Abstract
{
    public interface ICloudRepository : IBaseRepository<CloudModel>
    {
        Task<CloudModel> PatchCloudAsync(Guid id, CloudModel patch);
    }
}
