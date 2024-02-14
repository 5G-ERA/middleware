using Middleware.Models.Domain;
using Middleware.Models.Dto;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IInstanceRepository : IRedisRepository<InstanceModel, InstanceDto>
    {
        Task<InstanceModel> PatchInstanceAsync(Guid id, InstanceModel patch);

        Task<InstanceModel?> FindAlternativeInstance(Guid instance);
    }
}
