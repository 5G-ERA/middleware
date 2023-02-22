using Middleware.DataAccess.Repositories.Abstract;
using Middleware.DataAccess.Repositories.Redis;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Services;

public class ActionService : IActionService
{
    private readonly IActionRepository _actionRepository;
    public ActionService(IActionRepository actionRepository)
    {
        _actionRepository = actionRepository;
    }
    public async Task<ActionModel> AddAsync(ActionModel model)
    {
        var action = await _actionRepository.AddAsync(model);
        return action;
    }

    public async Task<ActionModel> GetByIdAsync(Guid id)
    {
        var action = await _actionRepository.GetByIdAsync(id);

        return action;
    }
}