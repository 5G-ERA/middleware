using Middleware.Common.Models;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IContainerImageRepository : IBaseRepository<ContainerImageModel>
    {
        Task<ContainerImageModel> PatchContainerImageAsync(Guid id, ContainerImageModel patch);
        /// <summary>
        /// Get images needed to be deployed by the action
        /// </summary>
        /// <param name="instanceId"></param>
        /// <returns></returns>
        Task<List<ContainerImageModel>> GetImagesForInstanceAsync(Guid instanceId);
    }
}
