using Middleware.Common.Models;

namespace Middleware.Common.Repositories.Abstract
{
    public interface ICloudRepository : IBaseRepository<CloudModel>
    {
        Task<CloudModel> PatchCloudAsync(Guid id, CloudModel patch);

        Task<CloudModel> GetCloudResourceDetailsByNameAsync(string name);
    }
}
