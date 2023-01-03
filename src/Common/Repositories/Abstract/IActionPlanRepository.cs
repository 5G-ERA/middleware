using Middleware.Common.Models;

namespace Middleware.Common.Repositories;

public interface IActionPlanRepository : IBaseRepository<ActionPlanModel>
{
    Task<List<ActionPlanModel>> GetActionPlanModelsAsync(Guid robotId);
}
