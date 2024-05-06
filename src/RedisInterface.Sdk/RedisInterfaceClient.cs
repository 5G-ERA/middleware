using Microsoft.Extensions.Logging;
using Middleware.Common.Result;
using Middleware.Models.Domain;
using Middleware.Models.ExtensionMethods;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Sdk.Client;

namespace Middleware.RedisInterface.Sdk;

public class RedisInterfaceClient : IRedisInterfaceClient
{
    private readonly IRedisInterface _api;
    private readonly ILogger<RedisInterfaceClient> _logger;

    public RedisInterfaceClient(IRedisInterface api, ILogger<RedisInterfaceClient> logger)
    {
        _api = api;
        _logger = logger;
    }

    public async Task<Result> AddRelationAsync<TSource, TDirection>(TSource source, TDirection direction, string name)
        where TSource : BaseModel where TDirection : BaseModel
    {
        if (source is null) throw new ArgumentNullException(nameof(source));
        if (direction is null) throw new ArgumentNullException(nameof(direction));
        if (name is null) throw new ArgumentNullException(nameof(name));

        var entity = source.GetType().GetModelName();
        var relation = CreateRelation(source, direction, name);
        var result = await _api.RelationAdd(entity, relation);

        return result.IsSuccessStatusCode ? Result.Success() : Result.Failure($"Could not create relation: {result.Error!.Message}");
    }

    public async Task<Result> DeleteRelationAsync<TSource, TDirection>(TSource source, TDirection direction,
        string name) where TSource : BaseModel where TDirection : BaseModel
    {
        if (source is null) throw new ArgumentNullException(nameof(source));
        if (direction is null) throw new ArgumentNullException(nameof(direction));
        if (name is null) throw new ArgumentNullException(nameof(name));

        var entity = source.GetType().GetModelName();
        var relation = CreateRelation(source, direction, name);

        var result = await _api.RelationDelete(entity, relation);
        if (result.IsSuccessStatusCode == false)
        {
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, Error: {error}", nameof(DeleteRelationAsync), result.Error!.Message);
            return $"Could not delete relation. {result.Error.Message}";
        }
        
        return Result.Success();
    }

    public Task<ActionPlanModel?> ActionPlanGetByIdAsync(Guid id)
    {
        return ActionPlanGetByIdAsync(id, CancellationToken.None);
    }

    public async Task<List<ActionPlanModel>?> ActionPlanGetAllAsync()
    {
        return await ActionPlanGetAllAsync(CancellationToken.None);
    }

    public async Task<bool> ActionPlanAddAsync(ActionPlanModel actionPlan)
    {
        return await ActionPlanAddAsync(actionPlan, CancellationToken.None);
    }

    public async Task<bool> ActionPlanDeleteAsync(Guid id)
    {
        return await ActionPlanDeleteAsync(id, CancellationToken.None);
    }

    public async Task<RobotResponse?> RobotGetByIdAsync(Guid id)
    {
        return await RobotGetByIdAsync(id, CancellationToken.None);
    }


    public async Task<TaskResponse?> TaskGetByIdAsync(Guid id)
    {
        return await TaskGetByIdAsync(id, CancellationToken.None);
    }

    public async Task<InstanceResponse?> GetInstanceAlternativeAsync(Guid id)
    {
        return await GetInstanceAlternativeAsync(id, CancellationToken.None);
    }

    public async Task<List<RelationModel>?> GetRelationForActionAsync(Guid id, string relationName)
    {
        var action = new ActionModel { Id = id };
        return await GetRelationAsync(action, relationName);
    }

    public async Task<List<RelationModel>?> GetRelationAsync<TSource>(TSource source, string relationName,
        string? direction = null)
        where TSource : BaseModel
    {
        if (source is null)
            throw new ArgumentNullException(nameof(source));
        if (relationName is null)
            throw new ArgumentNullException(nameof(relationName));


        var entity = source.GetType().GetModelName();
        var result = await _api.RelationGet(entity, relationName, source.Id, direction!);

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetRelationAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<InstanceResponse?> InstanceGetByIdAsync(Guid id)
    {
        return await InstanceGetByIdAsync(id, CancellationToken.None);
    }

    /// <inheritdoc />
    public async Task<GetInstancesResponse?> InstanceGetAllAsync()
    {
        var result = await _api.InstanceGetAll();

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(InstanceGetAllAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<GetContainersResponse?> ContainerImageGetForInstanceAsync(Guid id)
    {
        return await ContainerImageGetForInstanceAsync(id, CancellationToken.None);
    }

    public async Task<GetPoliciesResponse?> PolicyGetActiveAsync()
    {
        var result = await _api.PolicyGetActive();

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(PolicyGetActiveAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    [Obsolete("This function has little semantic sense. It should be replaced with something more meaningful ")]
    public async Task<GetCloudsResponse?> RobotGetConnectedCloudsAsync(Guid robotId)
    {
        if (robotId == default)
            throw new ArgumentNullException(nameof(robotId));

        var result = await _api.RobotGetConnectedClouds(robotId);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(RobotGetConnectedCloudsAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    [Obsolete("This function has little semantic sense. It should be replaced with something more meaningful ")]
    public async Task<GetCloudsResponse?> GetFreeCloudIdsAsync(List<CloudModel>? availableClouds)
    {
        if (availableClouds is null || availableClouds.Any() == false)
            throw new ArgumentException(nameof(availableClouds));

        var result = await _api.CloudGetFree(availableClouds);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetFreeCloudIdsAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    [Obsolete("This function has little semantic sense. It should be replaced with something more meaningful ")]
    public async Task<GetCloudsResponse?> GetLessBusyCloudsAsync(List<CloudModel> riconnectedClouds)
    {
        if (riconnectedClouds is null || riconnectedClouds.Any() == false)
            throw new ArgumentException(null, nameof(riconnectedClouds));

        var result = await _api.CloudGetLessBusy(riconnectedClouds);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetLessBusyCloudsAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<CloudResponse?> CloudGetByNameAsync(string cloudName)
    {
        if (string.IsNullOrWhiteSpace(cloudName))
            throw new ArgumentException("Value cannot be null or whitespace.", nameof(cloudName));

        var result = await _api.CloudGetByName(cloudName);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(CloudGetByNameAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    [Obsolete("This function has little semantic sense. It should be replaced with something more meaningful ")]
    public async Task<GetEdgesResponse?> RobotGetConnectedEdgesIdsAsync(Guid robotId)
    {
        if (robotId == default)
            throw new ArgumentNullException(nameof(robotId));

        var result = await _api.RobotGetConnectedEdges(robotId);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(RobotGetConnectedEdgesIdsAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    [Obsolete("This function has little semantic sense. It should be replaced with something more meaningful ")]
    public async Task<GetEdgesResponse?> GetFreeEdgesIdsAsync(List<EdgeModel> connectedEdges)
    {
        if (connectedEdges is null || connectedEdges.Any() == false)
            throw new ArgumentException(nameof(connectedEdges));

        var result = await _api.EdgeGetFree(connectedEdges);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetFreeEdgesIdsAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    [Obsolete("This function has little semantic sense. It should be replaced with something more meaningful ")]
    public async Task<GetEdgesResponse?> GetLessBusyEdgesAsync(List<EdgeModel> connectedEdges)
    {
        if (connectedEdges is null || connectedEdges.Any() == false)
            throw new ArgumentException(nameof(connectedEdges));

        var result = await _api.EdgeGetLessBusy(connectedEdges);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetLessBusyEdgesAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<EdgeResponse?> EdgeGetByNameAsync(string edgeName)
    {
        if (string.IsNullOrWhiteSpace(edgeName))
            throw new ArgumentException("Value cannot be null or whitespace.", nameof(edgeName));

        var result = await _api.EdgeGetByName(edgeName);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(EdgeGetByNameAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    /// <inheritdoc />
    public async Task<PolicyResponse?> GetPolicyByNameAsync(string policyName)
    {
        if (string.IsNullOrWhiteSpace(policyName))
            throw new ArgumentException("Value cannot be null or whitespace.", nameof(policyName));

        var result = await _api.PolicyGetByName(policyName);

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetPolicyByNameAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    /// <inheritdoc />
    public async Task<GetSlicesResponse?> SliceGetAllAsync()
    {
        var result = await _api.SliceGetAll();

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(SliceGetAllAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    /// <inheritdoc />
    public async Task<SliceResponse?> SliceGetByIdAsync(Guid id)
    {
        if (id == default)
            throw new ArgumentNullException(nameof(id));

        var result = await _api.SliceGetById(id);

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(SliceGetByIdAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }


    public async Task<ActionResponse?> ActionGetByIdAsync(Guid id)
    {
        return await ActionGetByIdAsync(id, CancellationToken.None);
    }

    public async Task<ActionPlanModel?> GetLatestActionPlanByRobotIdAsync(Guid robotId)
    {
        return await GetLatestActionPlanByRobotIdAsync(robotId, CancellationToken.None);
    }

    public async Task<EdgeResponse?> GetEdgeByNameAsync(string name)
    {
        return await GetEdgeByNameAsync(name, CancellationToken.None);
    }

    public Task<CloudResponse?> GetCloudByNameAsync(string name)
    {
        return GetCloudByNameAsync(name, CancellationToken.None);
    }

    public async Task<SliceResponse?> GetBySliceIdAsync(string id)
    {
        if (string.IsNullOrWhiteSpace(id))
            throw new ArgumentNullException(nameof(id));

        var result = await _api.GetBySliceIdAsync(id);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetBySliceIdAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<bool> SliceAddAsync(SliceRequest slice)
    {
        if (slice is null)
            throw new ArgumentNullException(nameof(slice));

        var result = await _api.SliceAddAsync(slice);

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(ActionPlanAddAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode;
    }

    /// <inheritdoc />
    public async Task<GetRobotsResponse?> RobotGetAllAsync()
    {
        var result = await _api.RobotGetAll();

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(RobotGetAllAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    /// <inheritdoc />
    public async Task<LocationResponse?> GetLocationByNameAsync(string name)
    {
        if (string.IsNullOrWhiteSpace(name))
            throw new ArgumentNullException(nameof(name));

        var result = await _api.LocationGetByName(name);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetLocationByNameAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    /// <inheritdoc />
    public async Task<LocationResponse?> GetLocationByIdAsync(Guid id)
    {
        if (id == Guid.Empty)
            throw new ArgumentNullException(nameof(id));

        var result = await _api.LocationGetById(id);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetLocationByIdAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    /// <inheritdoc />
    public async Task<GetLocationsResponse?> GetFreeLocationIdsAsync(List<Location>? availableLocations)
    {
        if (availableLocations is null || availableLocations.Any() == false)
            throw new ArgumentException(nameof(availableLocations));

        var result = await _api.LocationGetFree(availableLocations);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetFreeLocationIdsAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    /// <inheritdoc />
    public async Task<GetLocationsResponse?> GetLessBusyLocationsAsync(List<Location> availableLocations)
    {
        if (availableLocations is null || availableLocations.Any() == false)
            throw new ArgumentException(nameof(availableLocations));

        var result = await _api.LocationGetLessBusy(availableLocations);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetLessBusyLocationsAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    /// <inheritdoc />
    public async Task<GetLocationsResponse?> LocationGetAllAsync()
    {
        var result = await _api.LocationGetAll();
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetLessBusyLocationsAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    private RelationModel CreateRelation<TSource, TDirection>(TSource source, TDirection direction, string name)
        where TSource : BaseModel
        where TDirection : BaseModel
    {
        var initiatesFrom = new GraphEntityModel(source.Id, source.Name, source.GetType());
        var pointsTo = new GraphEntityModel(direction.Id, direction.Name, direction.GetType());
        return new(initiatesFrom, pointsTo, name);
    }

    public async Task<ActionPlanModel?> ActionPlanGetByIdAsync(Guid id, CancellationToken token)
    {
        if (id == default)
            throw new ArgumentNullException(nameof(id));

        var result = await _api.ActionPlanGetById(id);
        
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(ActionPlanGetByIdAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<List<ActionPlanModel>?> ActionPlanGetAllAsync(CancellationToken token)
    {
        var result = await _api.ActionPlanGetAllAsync();

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(ActionPlanGetAllAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<bool> ActionPlanAddAsync(ActionPlanModel actionPlan, CancellationToken token)
    {
        if (actionPlan is null)
            throw new ArgumentNullException(nameof(actionPlan));

        var result = await _api.ActionPlanAddAsync(actionPlan);

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(ActionPlanAddAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode;
    }

    public async Task<bool> ActionPlanDeleteAsync(Guid id, CancellationToken token)
    {
        if (id == default) throw new ArgumentNullException(nameof(id));

        await _api.ActionPlanDeleteAsync(id);

        return true;
    }

    public async Task<RobotResponse?> RobotGetByIdAsync(Guid robotId, CancellationToken token)
    {
        if (robotId == default)
            throw new ArgumentNullException(nameof(robotId));

        var result = await _api.RobotGetById(robotId);

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(RobotGetByIdAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<TaskResponse?> TaskGetByIdAsync(Guid taskId, CancellationToken token)
    {
        if (taskId == default)
            throw new ArgumentNullException(nameof(taskId));

        var result = await _api.TaskGetById(taskId);

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(TaskGetByIdAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<InstanceResponse?> GetInstanceAlternativeAsync(Guid instanceId, CancellationToken token)
    {
        if (instanceId == default)
            throw new ArgumentNullException(nameof(instanceId));

        var result = await _api.InstanceGetAlternative(instanceId);

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetInstanceAlternativeAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<InstanceResponse?> InstanceGetByIdAsync(Guid id, CancellationToken token)
    {
        if (id == default)
            throw new ArgumentNullException(nameof(id));

        var result = await _api.InstanceGetById(id);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(InstanceGetByIdAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<GetContainersResponse?> ContainerImageGetForInstanceAsync(Guid id, CancellationToken token)
    {
        if (id == default)
            throw new ArgumentNullException(nameof(id));

        var result = await _api.ContainerImageGetForInstance(id);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(ContainerImageGetForInstanceAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<ActionResponse?> ActionGetByIdAsync(Guid id, CancellationToken token)
    {
        if (id == default)
            throw new ArgumentNullException(nameof(id));

        var result = await _api.ActionGetById(id);

        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(ActionGetByIdAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<ActionPlanModel?> GetLatestActionPlanByRobotIdAsync(Guid robotId, CancellationToken token)
    {
        if (robotId == default)
            throw new ArgumentNullException(nameof(robotId));

        var result = await _api.ActionPlanGetLatestByRobotId(robotId);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetLatestActionPlanByRobotIdAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<EdgeResponse?> GetEdgeByNameAsync(string name, CancellationToken token)
    {
        if (string.IsNullOrWhiteSpace(name))
            throw new ArgumentNullException(nameof(name));

        var result = await _api.EdgeGetByName(name);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetEdgeByNameAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }

    public async Task<CloudResponse?> GetCloudByNameAsync(string name, CancellationToken token)
    {
        if (string.IsNullOrWhiteSpace(name))
            throw new ArgumentNullException(nameof(name));

        var result = await _api.CloudGetByName(name);
        if (!result.IsSuccessStatusCode)
            _logger.LogError(result.Error, "{funcName} - unsuccessful API call, StackTrace: {stackTrace}", nameof(GetCloudByNameAsync), Environment.StackTrace);

        return result.IsSuccessStatusCode ? result.Content : null;
    }
}