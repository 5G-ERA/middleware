using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface ITaskRepository : IBaseRepository<TaskModel>, IRelationRepository
    {
        Task<TaskModel> PatchTaskAsync(Guid id, TaskModel patch);
    }
}
