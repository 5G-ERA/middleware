using Middleware.DataAccess.Repositories.Abstract;
using Middleware.DataAccess.Repositories.Redis;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Services;

public class ActionRunningService : IActionRunningService
{
    private readonly IActionRunningRepository _actionRunningRepository;
    public ActionRunningService(IActionRunningRepository actionRunningRepository)
    {
        _actionRunningRepository = actionRunningRepository;
    }
    public async Task<ActionRunningModel> AddAsync(ActionRunningModel model)
    {
        var action = await _actionRunningRepository.AddAsync(model);
        return action;
    }

    public async Task<ActionRunningModel> GetByIdAsync(Guid id)
    {
        var action = await _actionRunningRepository.GetByIdAsync(id);

        return action;
    }
}