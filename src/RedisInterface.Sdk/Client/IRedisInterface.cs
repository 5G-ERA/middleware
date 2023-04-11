using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Responses;
using Refit;

namespace Middleware.RedisInterface.Sdk.Client;

internal interface IRedisInterface
{
    [Get("action/{id}")]
    Task<ApiResponse<ActionResponse?>> ActionGetById(Guid id);
    
    [Post("action/plan")]
    Task<ApiResponse<ActionPlanModel>> ActionPlanAddAsync([Body] ActionPlanModel payload);
    [Get("action/plan")]
    Task<ApiResponse<List<ActionPlanModel>?>> ActionPlanGetAllAsync();
    [Get("action/plan/{id}")]
    Task<ApiResponse<ActionPlanModel?>> ActionPlanGetById(Guid id);
    [Delete("action/plan")]
    Task ActionPlanDeleteAsync(Guid id);

    [Get("action/plan/robot/{id}")]
    Task<ApiResponse<ActionPlanModel?>> ActionPlanGetLatestByRobotId(Guid id);
    
    [Get("cloud/name/{name}")]
    Task<ApiResponse<CloudResponse?>> CloudGetByName(string name);

    [Post("cloud/free")]
    Task<ApiResponse<GetCloudsResponse>> CloudGetFree([Body] List<CloudModel> clouds);
    
    [Post("cloud/lessBusy")]
    Task<ApiResponse<GetCloudsResponse>> CloudGetLessBusy([Body] List<CloudModel> clouds);
    
    [Get("containerImage/instance/{id}")]
    Task<ApiResponse<GetContainersResponse?>> ContainerImageGetForInstance(Guid id);
    
    [Get("edge/name/{name}")]
    Task<ApiResponse<EdgeResponse?>> EdgeGetByName(string name);
    
    [Post("edge/free")]
    Task<ApiResponse<GetEdgesResponse>> EdgeGetFree([Body] List<EdgeModel> edges);
    
    [Post("edge/lessBusy")]
    Task<ApiResponse<GetEdgesResponse>> EdgeGetLessBusy([Body] List<EdgeModel> edges);
    
    [Get("instance/alternative/{id}")]
    Task<ApiResponse<InstanceResponse?>> InstanceGetById(Guid id);
    [Get("instance/alternative/{id}")]
    Task<ApiResponse<InstanceResponse>> InstanceGetAlternative(Guid id);

    [Get("policy/current")]
    Task<ApiResponse<GetPoliciesResponse>> PolicyGetActive();
    
    
    [Post("{entity}/AddRelation")]
    Task RelationAdd(string entity, [Body]RelationModel payload);
    
    [Post("{entity}/relation/{name}")]
    Task<ApiResponse<List<RelationModel>?>> RelationGet(string entity, [AliasAs("name")] string relationName, Guid id);
    
    [Delete("{entity}/DeleteRelation")]
    Task RelationDelete(string entity, [Body]RelationModel payload);
    
    [Get("robot/{id}")]
    Task<ApiResponse<RobotResponse?>> RobotGetById(Guid id);
    
    [Get("robot/{id}/clouds/connected")]
    Task<ApiResponse<GetCloudsResponse?>> RobotGetConnectedClouds(Guid id);
    
    
    [Get("robot/{id}/edges/connected")]
    Task<ApiResponse<GetEdgesResponse?>> RobotGetConnectedEdges(Guid id);
    
    [Get("task/{id}")]
    Task<ApiResponse<TaskResponse?>> TaskGetById(Guid id);

    
    
    

    
}