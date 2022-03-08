using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories
{
    public interface ITaskRepository : IBaseRepository<TaskModel>
    {
        Task<List<TaskModel>> GetTaskAsync();

        Task<TaskModel> GetTaskIdAsync(string id);

        Task<TaskModel> GetTaskRelationshipsAsync();


    }
}
