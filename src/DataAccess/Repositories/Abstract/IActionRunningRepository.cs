using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IActionRunningRepository: IBaseRepository<ActionRunningModel>, IRelationRepository
    {
        Task<ActionRunningModel> PatchActionAsync(Guid id, ActionRunningModel patch);
    }
}
