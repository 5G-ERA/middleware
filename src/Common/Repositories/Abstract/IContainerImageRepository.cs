using Middleware.Common.Models;

namespace Middleware.Common.Repositories.Abstract
{
    public interface IContainerImageRepository : IBaseRepository<ContainerImageModel>
    {
        Task<ContainerImageModel> PatchContainerImageAsync(Guid id, ContainerImageModel patch);
    }
}
