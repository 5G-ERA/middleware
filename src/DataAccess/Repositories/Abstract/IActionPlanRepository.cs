using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface IActionPlanRepository : IBaseRepository<ActionPlanModel>, IRelationRepository
{
    Task<List<ActionPlanModel>> GetRobotActionPlans(Guid robotId);
}