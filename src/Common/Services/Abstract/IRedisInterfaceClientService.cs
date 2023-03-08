using Middleware.Models.Domain;

namespace Middleware.Common.Services;

public interface IRedisInterfaceClientService
{
    /// <summary>
    /// Get <see cref="ActionPlanModel"/> by its id
    /// </summary>
    /// <param name="id">Unique identifier of <see cref="ActionPlanModel"/></param>
    /// <returns>Complete action plan</returns>
    /// 
    Task<ActionPlanModel> ActionPlanGetByIdAsync(Guid id);
    /// <summary>
    /// Get <see cref="ActionPlanModel"/> by its id
    /// </summary>
    /// <param name="id">Unique identifier of <see cref="ActionPlanModel"/></param>
    /// <param name="token">Cancellation token</param>
    /// <returns>Complete action plan</returns>
    Task<ActionPlanModel> ActionPlanGetByIdAsync(Guid id, CancellationToken token);
    
    Task<List<ActionPlanModel>> ActionPlanGetAllAsync();
    Task<List<ActionPlanModel>> ActionPlanGetAllAsync(CancellationToken token);

    Task<bool> ActionPlanAddAsync(ActionPlanModel actionPlan);
    Task<bool> ActionPlanAddAsync(ActionPlanModel actionPlan, CancellationToken token);
    Task<bool> ActionPlanDeleteAsync(Guid id);
    Task<bool> ActionPlanDeleteAsync(Guid id, CancellationToken token);

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
    /// Gets the alternative for the specified instance
    /// </summary>
    /// <param name="id">Id of an instance</param>
    /// <returns>The instance that can be treated as alternative to the given instance</returns>
    Task<InstanceModel> GetInstanceAlternativeAsync(Guid id);
    /// <summary>
    /// Gets the alternative for the specified instance
    /// </summary>
    /// <param name="id">Id of an instance</param>
    /// <returns>The instance that can be treated as alternative to the given instance</returns>
    Task<InstanceModel> GetInstanceAlternativeAsync(Guid id, CancellationToken token);

    /// <summary>
    /// Create graph relation between objects
    /// </summary>
    /// <param name="source"></param>
    /// <param name="direction"></param>
    /// <param name="name"></param>
    /// <typeparam name="TSource">Object that derives from <see cref="BaseModel"/></typeparam>
    /// <typeparam name="TDirection">Object that derives from <see cref="BaseModel"/></typeparam>
    /// <returns>Have relation been created</returns>
    Task<bool> AddRelationAsync<TSource, TDirection>(TSource source, TDirection direction, string name)
        where TSource : BaseModel where TDirection : BaseModel;
    
    Task<bool> DeleteRelationAsync<TSource, TDirection>(TSource source, TDirection direction, string name)
        where TSource : BaseModel where TDirection : BaseModel;

    /// <summary>
    /// Get relations with the specified name for action
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    Task<List<RelationModel>> GetRelationForActionAsync(Guid id, string relationName);

    /// <summary>
    /// Get the relations with the specified name that are outcoming from the specified object
    /// </summary>
    /// <typeparam name="TSource"></typeparam>
    /// <param name="relationName"></param>
    /// <returns></returns>
    Task<List<RelationModel>> GetRelationAsync<TSource>(TSource source, string relationName) where TSource : BaseModel;

    /// <summary>
    /// Get Action by its id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    Task<ActionModel> ActionGetByIdAsync(Guid id);

    /// <summary>
    /// Get Action by its id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    Task<ActionModel> ActionGetByIdAsync(Guid id, CancellationToken token);

    /// <summary>
    /// Get the latest action plan that robot has executed
    /// </summary>
    /// <param name="robotId">Robot Id</param>
    /// <returns></returns>
    Task<ActionPlanModel> GetLatestActionPlanByRobotIdAsync(Guid robotId);
    /// <summary>
    /// Get the latest action plan that robot has executed
    /// </summary>
    /// <param name="robotId">Robot Id</param>
    /// <returns></returns>
    Task<ActionPlanModel> GetLatestActionPlanByRobotIdAsync(Guid robotId, CancellationToken token);
    /// <summary>
    /// Get Edge Data by its name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<EdgeModel> GetEdgeByNameAsync(string name);
    /// <summary>
    /// Get Edge Data by its name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<EdgeModel> GetEdgeByNameAsync(string name, CancellationToken token);
    /// <summary>
    /// Get Cloud Data by its name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<CloudModel> GetCloudByNameAsync(string name);
    /// <summary>
    /// Get Cloud Data by its name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<CloudModel> GetCloudByNameAsync(string name, CancellationToken token);

    /// <summary>
    /// Get instance data by its id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    Task<InstanceModel> InstanceGetByIdAsync(Guid id);

    /// <summary>
    /// Get instance data by its id
    /// </summary>
    /// <param name="id"></param>
    /// <param name="token"></param>
    /// <returns></returns>
    Task<InstanceModel> InstanceGetByIdAsync(Guid id, CancellationToken token);

    Task<List<ContainerImageModel>> ContainerImageGetForInstanceAsync(Guid id);
    Task<List<ContainerImageModel>> ContainerImageGetForInstanceAsync(Guid id, CancellationToken token);

    Task<bool> ActionRunningAddAsync(ActionRunningModel actionRunning);

    Task<bool> ActionRunningAddAsync(ActionRunningModel actionRunning, CancellationToken token);

    Task<bool> InstanceRunningAddAsync(InstanceRunningModel instanceRunning);

    Task<bool> InstanceRunningAddAsync(InstanceRunningModel instanceRunning, CancellationToken token);

    Task<bool> HistoricalActionPlanAddAsync(HistoricalActionPlanModel historicalActionPlan);

    Task<bool> HistoricalActionPlanAddAsync(HistoricalActionPlanModel historicalActionPlan, CancellationToken token);
}