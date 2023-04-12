using Middleware.Models.Domain;
using Middleware.RedisInterface.Contracts.Requests;
using Middleware.RedisInterface.Contracts.Responses;
using Refit;

namespace Middleware.RedisInterface.Sdk.Client;

internal interface IRedisInterface
{
    [Get("/api/v1/action/{id}")]
    Task<ApiResponse<ActionResponse?>> ActionGetById(Guid id);
    
    [Post("/api/v1/action/plan")]
    Task<ApiResponse<ActionPlanModel>> ActionPlanAddAsync([Body] ActionPlanModel payload);
    [Get("/api/v1/action/plan")]
    Task<ApiResponse<List<ActionPlanModel>?>> ActionPlanGetAllAsync();
    [Get("/api/v1/action/plan/{id}")]
    Task<ApiResponse<ActionPlanModel?>> ActionPlanGetById(Guid id);
    [Delete("/api/v1/action/plan")]
    Task ActionPlanDeleteAsync(Guid id);

    [Get("/api/v1/action/plan/robot/{id}")]
    Task<ApiResponse<ActionPlanModel?>> ActionPlanGetLatestByRobotId(Guid id);
    
    [Get("/api/v1/cloud/name/{name}")]
    Task<ApiResponse<CloudResponse?>> CloudGetByName(string name);
    
    [Post("/api/v1/cloud")]
    Task<ApiResponse<CloudResponse?>> CloudAdd([Body]CloudRequest payload);

    [Post("/api/v1/cloud/free")]
    Task<ApiResponse<GetCloudsResponse>> CloudGetFree([Body] List<CloudModel> clouds);
    
    [Post("/api/v1/cloud/lessBusy")]
    Task<ApiResponse<GetCloudsResponse>> CloudGetLessBusy([Body] List<CloudModel> clouds);
    
    [Get("/api/v1/containerImage/instance/{id}")]
    Task<ApiResponse<GetContainersResponse?>> ContainerImageGetForInstance(Guid id);
    
    [Get("/api/v1/edge/name/{name}")]
    Task<ApiResponse<EdgeResponse?>> EdgeGetByName(string name);
    
    [Post("/api/v1/edge/free")]
    Task<ApiResponse<GetEdgesResponse>> EdgeGetFree([Body] List<EdgeModel> edges);
    
    [Post("/api/v1/edge/lessBusy")]
    Task<ApiResponse<GetEdgesResponse>> EdgeGetLessBusy([Body] List<EdgeModel> edges);
    
    [Get("/api/v1/instance/alternative/{id}")]
    Task<ApiResponse<InstanceResponse?>> InstanceGetById(Guid id);
    [Get("/api/v1/instance/alternative/{id}")]
    Task<ApiResponse<InstanceResponse>> InstanceGetAlternative(Guid id);

    [Get("/api/v1/policy/current")]
    Task<ApiResponse<GetPoliciesResponse>> PolicyGetActive();
    
    [Post("/api/v1/{entity}/AddRelation")]
    Task RelationAdd(string entity, [Body]RelationModel payload);
    
    [Get("/api/v1/{entity}/relation/{name}")]
    Task<ApiResponse<List<RelationModel>?>> RelationGet(string entity, [AliasAs("name")] string relationName, Guid id);
    
    [Delete("/api/v1/{entity}/DeleteRelation")]
    Task RelationDelete(string entity, [Body]RelationModel payload);
    
    [Get("/api/v1/robot/{id}")]
    Task<ApiResponse<RobotResponse?>> RobotGetById(Guid id);
    
    [Get("/api/v1/robot/{id}/clouds/connected")]
    Task<ApiResponse<GetCloudsResponse?>> RobotGetConnectedClouds(Guid id);
    
    
    [Get("/api/v1/robot/{id}/edges/connected")]
    Task<ApiResponse<GetEdgesResponse?>> RobotGetConnectedEdges(Guid id);
    
    [Get("/api/v1/task/{id}")]
    Task<ApiResponse<TaskResponse?>> TaskGetById(Guid id);

    
    
    

    
}