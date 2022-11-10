using Middleware.Common.Models;
using Middleware.Common.Structs;

namespace Middleware.TaskPlanner.Services;

public interface IRedisInterfaceClientService
{
    /// <summary>
    /// Get <see cref="ActionPlanModel"/> by its id
    /// </summary>
    /// <param name="id">Unique identifier of <see cref="ActionPlanModel"/></param>
    /// <returns>Complete action plan</returns>
    Task<Either<ActionPlanModel, InvalidOperationException>> ActionPlanGetByIdAsync(Guid id);
    /// <summary>
    /// Get <see cref="ActionPlanModel"/> by its id
    /// </summary>
    /// <param name="id">Unique identifier of <see cref="ActionPlanModel"/></param>
    /// <param name="token">Cancellation token</param>
    /// <returns>Complete action plan</returns>
    Task<Either<ActionPlanModel, InvalidOperationException>> ActionPlanGetByIdAsync(Guid id, CancellationToken token);

    /// <summary>
    /// Gets robot data by id
    /// </summary>
    /// <param name="id">Identifier of a robot</param>
    /// <returns>Robot data</returns>
    Task<RobotModel> RobotGetByIdAsync(Guid id);

    /// <summary>
    /// Gets robot data by id
    /// </summary>
    /// <param name="id">Identifier of a robot</param>
    /// <param name="token">Token</param>
    /// <returns>Robot data</returns>
    Task<RobotModel> RobotGetByIdAsync(Guid id, CancellationToken token);
    
    /// <summary>
    /// Gets task definition by id
    /// </summary>
    /// <param name="id">Identifier of a robot</param>
    /// <returns>Robot data</returns>
    Task<TaskModel> TaskGetByIdAsync(Guid id);

    /// <summary>
    /// Gets task definition by id
    /// </summary>
    /// <param name="id">Identifier of a task</param>
    /// <param name="token">Token</param>
    /// <returns>Task data</returns>
    Task<TaskModel> TaskGetByIdAsync(Guid id, CancellationToken token);
    
    /// <summary>
    /// Create graph relation between objects
    /// </summary>
    /// <param name="source"></param>
    /// <param name="direction"></param>
    /// <param name="name"></param>
    /// <typeparam name="TSource">Object that derives from <see cref="BaseModel"/></typeparam>
    /// <typeparam name="TDirection">Object that derives from <see cref="BaseModel"/></typeparam>
    /// <returns>Have relation been created</returns>
    Task<Either<bool, Exception>> AddRelation<TSource, TDirection>(TSource source, TDirection direction, string name)
        where TSource : BaseModel where TDirection : BaseModel;
    
    
}