using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories
{
    public interface ITaskRepository : IBaseRepository<TaskModel>
    {
        Task<TaskModel> PatchTaskAsync(Guid id, TaskModel patch);
    }
}
