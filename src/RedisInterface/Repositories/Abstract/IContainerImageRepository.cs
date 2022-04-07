using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories
{
    public interface IContainerImageRepository : IBaseRepository<ContainerImageModel>
    {
        Task<ContainerImageModel> PatchContainerImageAsync(Guid id, ContainerImageModel patch);
        /// <summary>
        /// Get images needed to be deployed by the action
        /// </summary>
        /// <param name="actionId"></param>
        /// <returns></returns>
        Task<List<ContainerImageModel>> GetImagesForActionAsync(Guid actionId);
    }
}
