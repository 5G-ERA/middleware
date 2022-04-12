using Middleware.Common.Models;

namespace Middleware.Common.Repositories.Abstract
{
    public interface IActionRepository : IBaseRepository<ActionModel>
    {
        Task<ActionModel> PatchActionAsync(Guid id, ActionModel patch);

    }
}
