using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories.Abstract
{
    public interface IContainerImageRepository : IBaseRepository<ContainerImageModel>
    {
        Task<ContainerImageModel> PatchContainerImageAsync(Guid id, ContainerImageModel patch);

        Task<List<RelationModel>> GetRelation(Guid id, string relationName);
    }
}
