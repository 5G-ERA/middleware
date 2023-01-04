using Middleware.Common.Models;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IActionRepository : IBaseRepository<ActionModel>
    {
        Task<ActionModel> PatchActionAsync(Guid id, ActionModel patch);

    }
}
