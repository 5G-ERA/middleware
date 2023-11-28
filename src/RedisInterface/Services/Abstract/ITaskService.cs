using Middleware.Models.Domain;

namespace Middleware.RedisInterface.Services.Abstract;

public interface ITaskService
{
    /// <summary>
    ///     Return a full definition of a task including associated Actions, Instances and Container Images
    /// </summary>
    /// <returns></returns>
    Task<TaskModel> GetFullTaskDefinitionAsync(Guid id);
}