using Middleware.Common.Models;

namespace Middleware.RedisInterface.Repositories
{
    public interface ITaskRepository : IBaseRepository<TaskModel>
    {

        Task<List<RelationModel>> GetRelation(Guid id, string relationName);

        Task<TaskModel> PatchTaskAsync(Guid id, TaskModel patch);

    }
}
