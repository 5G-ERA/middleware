using Middleware.Common.Result;
using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;

namespace Middleware.RedisInterface.Sdk;

public interface IRedisInterfaceClient
{
    /// <summary>
    ///     Get <see cref="ActionPlanModel" /> by its id
    /// </summary>
    /// <param name="id">Unique identifier of <see cref="ActionPlanModel" /></param>
    /// <returns>Complete action plan</returns>
    Task<ActionPlanModel?> ActionPlanGetByIdAsync(Guid id);

    Task<List<ActionPlanModel>?> ActionPlanGetAllAsync();

    Task<bool> ActionPlanAddAsync(ActionPlanModel actionPlan);
    Task<bool> ActionPlanDeleteAsync(Guid id);

    /// <summary>
    ///     Gets robot data by id
    /// </summary>
    /// <param name="id">Identifier of a robot</param>
    /// <returns>Robot data</returns>
    Task<RobotResponse?> RobotGetByIdAsync(Guid id);

    /// <summary>
    ///     Gets task definition by id
    /// </summary>
    /// <param name="id">Identifier of a robot</param>
    /// <returns>Robot data</returns>
    Task<TaskResponse?> TaskGetByIdAsync(Guid id);

    /// <summary>
    ///     Gets the alternative for the specified instance
    /// </summary>
    /// <param name="id">Id of an instance</param>
    /// <returns>The instance that can be treated as alternative to the given instance</returns>
    Task<InstanceResponse?> GetInstanceAlternativeAsync(Guid id);

    /// <summary>
    ///     Create graph relation between objects
    /// </summary>
    /// <param name="source">Initiating point of a relation</param>
    /// <param name="direction">Receiving point of a relation</param>
    /// <param name="name">Name of the relation</param>
    /// <typeparam name="TSource">Object that derives from <see cref="BaseModel" /></typeparam>
    /// <typeparam name="TDirection">Object that derives from <see cref="BaseModel" /></typeparam>
    /// <returns>Have relation been created</returns>
    Task<Result> AddRelationAsync<TSource, TDirection>(TSource source, TDirection direction, string name)
        where TSource : BaseModel where TDirection : BaseModel;

    Task<Result> DeleteRelationAsync<TSource, TDirection>(TSource source, TDirection direction, string name)
        where TSource : BaseModel where TDirection : BaseModel;

    /// <summary>
    ///     Get relations with the specified name for action
    /// </summary>
    /// <param name="id">Identifier of the action</param>
    /// <param name="relationName">Name of the relation to be retrieved</param>
    /// <returns></returns>
    Task<List<RelationModel>?> GetRelationForActionAsync(Guid id, string relationName);

    /// <summary>
    ///     Get the relations with the specified name that are out-coming from the specified object
    /// </summary>
    /// <typeparam name="TSource">Type of the source object</typeparam>
    /// <param name="source">Source object that relation initiates from</param>
    /// <param name="relationName">Name of the relation to be retrieved</param>
    /// <param name="direction">Direction of the relation, incoming or outgoing</param>
    /// <returns></returns>
    Task<List<RelationModel>?> GetRelationAsync<TSource>(TSource source, string relationName, string? direction = null)
        where TSource : BaseModel;

    /// <summary>
    ///     Get Action by its id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    Task<ActionResponse?> ActionGetByIdAsync(Guid id);

    /// <summary>
    ///     Get the latest action plan that robot has executed
    /// </summary>
    /// <param name="robotId">Robot Id</param>
    /// <returns></returns>
    Task<ActionPlanModel?> GetLatestActionPlanByRobotIdAsync(Guid robotId);

    /// <summary>
    ///     Get Edge Data by its name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<EdgeResponse?> GetEdgeByNameAsync(string name);

    /// <summary>
    ///     Get Cloud Data by its name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<CloudResponse?> GetCloudByNameAsync(string name);

    /// <summary>
    ///     Get instance data by its id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    Task<InstanceResponse?> InstanceGetByIdAsync(Guid id);

    /// <summary>
    ///     Get all instance data
    /// </summary>
    /// <returns></returns>
    Task<GetInstancesResponse?> InstanceGetAllAsync();

    /// <summary>
    ///     Retrieves the Container Images associated with the specified Instance
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    Task<GetContainersResponse?> ContainerImageGetForInstanceAsync(Guid id);

    /// <summary>
    ///     Get Active Global policies in the Middleware
    /// </summary>
    /// <returns></returns>
    Task<GetPoliciesResponse?> PolicyGetActiveAsync();

    /// <summary>
    ///     Get Clouds that the robot is connected to
    /// </summary>
    /// <param name="robotId"></param>
    /// <returns></returns>
    Task<GetCloudsResponse?> RobotGetConnectedCloudsAsync(Guid robotId);

    /// <summary>
    ///     Pick only free (unused) edges from the specified list
    /// </summary>
    /// <param name="availableClouds"></param>
    /// <returns></returns>
    Task<GetCloudsResponse?> GetFreeCloudIdsAsync(List<CloudModel>? availableClouds);

    /// <summary>
    ///     Order Clouds based on the resource usage from the specified list
    /// </summary>
    /// <param name="availableClouds"></param>
    /// <returns></returns>
    Task<GetCloudsResponse?> GetLessBusyCloudsAsync(List<CloudModel> availableClouds);

    /// <summary>
    ///     Get Cloud by name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<CloudResponse?> CloudGetByNameAsync(string name);

    /// <summary>
    ///     Get Edges available to the robot
    /// </summary>
    /// <param name="robotId"></param>
    /// <returns></returns>
    Task<GetEdgesResponse?> RobotGetConnectedEdgesIdsAsync(Guid robotId);

    /// <summary>
    ///     Get unused edges from the available list
    /// </summary>
    /// <param name="connectedEdges"></param>
    /// <returns></returns>
    Task<GetEdgesResponse?> GetFreeEdgesIdsAsync(List<EdgeModel> connectedEdges);

    /// <summary>
    ///     Get Edges ordered by the least used from the given list
    /// </summary>
    /// <param name="connectedEdges"></param>
    /// <returns></returns>
    Task<GetEdgesResponse?> GetLessBusyEdgesAsync(List<EdgeModel> connectedEdges);

    /// <summary>
    ///     Get Edge by name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<EdgeResponse?> EdgeGetByNameAsync(string name);

    /// <summary>
    ///     Get Policy data by name
    /// </summary>
    /// <param name="policyName">name of the policy</param>
    /// <returns></returns>
    Task<PolicyResponse?> GetPolicyByNameAsync(string policyName);

    /// <summary>
    ///     Get all slices
    /// </summary>
    /// <returns></returns>
    Task<GetSlicesResponse?> SliceGetAllAsync();

    /// <summary>
    ///     Get Slice by Id
    /// </summary>
    /// <returns></returns>
    Task<SliceResponse?> SliceGetByIdAsync(Guid id);

    Task<SliceResponse?> GetBySliceIdAsync(string id);

    Task<bool> SliceAddAsync(SliceRequest slice);
    Task<GetRobotsResponse?> RobotGetAllAsync();

    /// <summary>
    ///     Get Location Data by its name
    /// </summary>
    /// <param name="name"></param>
    /// <returns></returns>
    Task<LocationResponse?> GetLocationByNameAsync(string name);

    /// <summary>
    ///     Get Location Data by its Id
    /// </summary>
    /// <param name="id"></param>
    /// <returns></returns>
    Task<LocationResponse?> GetLocationByIdAsync(Guid id);

    /// <summary>
    ///     Pick only free (unused) locations from the specified list
    /// </summary>
    /// <param name="availableLocations"></param>
    /// <returns></returns>
    Task<GetLocationsResponse?> GetFreeLocationIdsAsync(List<Location>? availableLocations);

    /// <summary>
    ///     Order Locations based on the resource usage from the specified list
    /// </summary>
    /// <param name="availableLocations"></param>
    /// <returns></returns>
    Task<GetLocationsResponse?> GetLessBusyLocationsAsync(List<Location> availableLocations);

    /// <summary>
    ///     Get All locations
    /// </summary>
    /// <returns></returns>
    Task<GetLocationsResponse?> LocationGetAllAsync();
}