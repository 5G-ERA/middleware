using Middleware.Models.Domain;
using Middleware.Models.Dto;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IRobotRepository : IRedisRepository<RobotModel, RobotDto>
    {
        Task<RobotModel> PatchRobotAsync(Guid id, RobotModel patch);

        Task<List<EdgeModel>> GetConnectedEdgesIdsAsync(Guid robotId);

        Task<List<CloudModel>> GetConnectedCloudsIdsAsync(Guid robotID);

    }
}
