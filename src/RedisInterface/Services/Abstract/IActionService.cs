using Middleware.Models.Domain;

namespace Middleware.RedisInterface.Services.Abstract;

public interface IActionService
{
    Task<ActionModel> GetByIdAsync(Guid id);
    Task<ActionModel> AddAsync(ActionModel model);
    Task UpdateAsync(ActionModel model);
    Task<bool> DeleteAsync(Guid id);
}
