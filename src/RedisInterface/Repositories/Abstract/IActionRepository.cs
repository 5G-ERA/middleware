using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories.Abstract
{
    public interface IActionRepository : IBaseRepository<ActionModel>
    {
        Task<ActionModel> PatchActionAsync(Guid id);
    }
}
