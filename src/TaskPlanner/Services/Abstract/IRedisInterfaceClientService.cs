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