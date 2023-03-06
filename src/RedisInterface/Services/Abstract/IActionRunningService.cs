using Middleware.Models.Domain;

namespace Middleware.RedisInterface.Services.Abstract;

public interface IActionRunningService
{
    Task<ActionRunningModel> GetByIdAsync(Guid id);
    Task<ActionRunningModel> AddAsync(ActionRunningModel model);
}
