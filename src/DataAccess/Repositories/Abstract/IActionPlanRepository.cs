using Middleware.Common.Models;

namespace DataAccess.Repositories.Abstract;

public interface IActionPlanRepository : IBaseRepository<ActionPlanModel>
{
    Task<List<ActionPlanModel>> GetActionPlanModelsAsync(Guid robotId);
}