using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IActionRepository : IBaseRepository<ActionModel>, IRelationRepository
    {
        Task<ActionModel> PatchActionAsync(Guid id, ActionModel patch);

    }
}
