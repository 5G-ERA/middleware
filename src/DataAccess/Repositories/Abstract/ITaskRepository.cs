using Middleware.Common.Models;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface ITaskRepository : IBaseRepository<TaskModel>
    {
        Task<TaskModel> PatchTaskAsync(Guid id, TaskModel patch);
    }
}
