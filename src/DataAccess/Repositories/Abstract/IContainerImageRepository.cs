using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IContainerImageRepository : IBaseRepository<ContainerImageModel>, IRelationRepository
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
