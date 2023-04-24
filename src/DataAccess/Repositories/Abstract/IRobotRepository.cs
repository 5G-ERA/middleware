using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IRobotRepository : IBaseRepository<RobotModel>, IRelationRepository
    {
        Task<RobotModel> PatchRobotAsync(Guid id, RobotModel patch);

        Task<List<EdgeModel>> GetConnectedEdgesIdsAsync(Guid robotId);

        Task<List<CloudModel>> GetConnectedCloudsIdsAsync(Guid robotID);

    }
}
