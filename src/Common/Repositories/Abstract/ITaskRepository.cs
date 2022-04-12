using Middleware.Common.Models;

namespace Middleware.Common.Repositories
{
    public interface ITaskRepository : IBaseRepository<TaskModel>
    {
        Task<TaskModel> PatchTaskAsync(Guid id, TaskModel patch);
    }
}
