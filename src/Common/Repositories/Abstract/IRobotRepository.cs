using Middleware.Common.Models;

namespace Middleware.Common.Repositories.Abstract
{
    public interface IRobotRepository : IBaseRepository<RobotModel>
    {
        Task<RobotModel> PatchRobotAsync(Guid id, RobotModel patch);

        Task<List<Guid>> GetConnectedEdgesIdsAsync(Guid robotId);

        Task<List<Guid>> GetFreeEdgesIdsAsync(List<Guid> listofEdgesConnectedtoRobot);
    }
}
