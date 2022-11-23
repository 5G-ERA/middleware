using Middleware.Common.Enums;
using Middleware.Common.Models;

namespace Middleware.Common.Repositories.Abstract
{
    public interface IRobotRepository : IBaseRepository<RobotModel>
    {
        Task<RobotModel> PatchRobotAsync(Guid id, RobotModel patch);

        Task<List<EdgeModel>> GetConnectedEdgesIdsAsync(Guid robotId);

        Task<List<CloudModel>> GetConnectedCloudsIdsAsync(Guid robotID);

    }
}
