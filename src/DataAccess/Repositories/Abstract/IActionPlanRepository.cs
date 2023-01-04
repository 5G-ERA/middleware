using Middleware.Common.Models;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface IActionPlanRepository : IBaseRepository<ActionPlanModel>
{
    Task<List<ActionPlanModel>> GetActionPlanModelsAsync(Guid robotId);
}