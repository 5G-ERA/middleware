using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories
{
    public interface ITaskRepository : IBaseRepository<TaskModel>
    {
        Task<List<TaskModel>> GetAllTasksAsync(Guid id);

        Task<TaskModel> GetTaskByIdAsync(Guid id);

        Task<TaskModel> GetTaskRelationshipsAsync();


    }
}
