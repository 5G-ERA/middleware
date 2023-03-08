using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface IHistoricalActionPlanRepository : IBaseRepository<HistoricalActionPlanModel>, IRelationRepository
{
    Task<List<HistoricalActionPlanModel>> GetRobotActionPlans(Guid robotId);

    Task<List<HistoricalActionPlanModel>> GetRobotReplanActionPlans(Guid robotId);
}