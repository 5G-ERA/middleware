using Middleware.Common.Models;
using Middleware.DataAccess.Dto;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IActionRepository : IBaseRepository<ActionModel>, IRelationRepository
    {
        Task<ActionModel> PatchActionAsync(Guid id, ActionModel patch);

    }
}
