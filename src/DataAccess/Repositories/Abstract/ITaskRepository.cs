using Middleware.Models.Domain;
using Middleware.Models.Dto;
using StackExchange.Redis;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface ITaskRepository : IRedisRepository<TaskModel, TaskDto>
    {
        Task<TaskModel> PatchTaskAsync(Guid id, TaskModel patch);
    }
}
