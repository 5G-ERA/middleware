using Middleware.Models.Domain;
using Middleware.Models.Dto;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IActionRepository : IRedisRepository<ActionModel, ActionDto>
    {
        Task<ActionModel> PatchActionAsync(Guid id, ActionModel patch);

    }
}
