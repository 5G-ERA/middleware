using Middleware.Models.Domain;
using Middleware.Models.Dto;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IContainerImageRepository : IRedisRepository<ContainerImageModel, ContainerImageDto>
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
