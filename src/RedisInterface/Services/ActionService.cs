using Middleware.Common.Models;
using Middleware.DataAccess.Repositories.Redis;
using Middleware.RedisInterface.Services.Abstract;

namespace Middleware.RedisInterface.Services
{
    public class ActionService : IActionService
    {
        private readonly RedisActionRepository _actionRepository;
        public ActionService(RedisActionRepository actionRepository)
        {
            _actionRepository = actionRepository;
        }
        public async Task<ActionModel> AddAsync(ActionModel model)
        {
            var dto = model.ToActionDto();

            var action = await _actionRepository.AddAsync(dto);

            return action.ToActionModel();
        }

        public async Task<ActionModel> GetByIdAsync(Guid id)
        {
            var dto = await _actionRepository.GetByIdAsync(id);

            return dto?.ToActionModel();
        }
    }
}
