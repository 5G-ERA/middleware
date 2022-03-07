using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories
{
    public interface ITaskRepository
    {
        Task<List<TaskModel>> GetTaskAsync();

        Task<TaskModel> GetTaskIdAsync(string id);

        Task<TaskModel> GetTaskRelationshipsAsync();


    }
}
