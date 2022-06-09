using System.Net;
using AutoMapper;
using Middleware.Common;
using Middleware.Common.Models;
using Middleware.ResourcePlanner.ApiReference;

namespace Middleware.ResourcePlanner;

public interface IResourcePlanner
{
    Task<TaskModel> Plan(TaskModel taskModel);
}

public class ResourcePlanner : IResourcePlanner
{
    private readonly IApiClientBuilder _apiClientBuilder;
    private readonly IMapper _mapper;
    private readonly IEnvironment _env;
    private readonly ILogger _logger;

    public ResourcePlanner(IApiClientBuilder apiClientBuilder, IMapper mapper, IEnvironment env, ILogger<ResourcePlanner> logger)
    {
        _apiClientBuilder = apiClientBuilder;
        _mapper = mapper;
        _env = env;
        _logger = logger;
    }
        
    public async Task<TaskModel> Plan(TaskModel taskModel)
    {
            
        var redisApiClient = _apiClientBuilder.CreateRedisApiClient();
        var orchestratorApiClient = _apiClientBuilder.CreateOrchestratorApiClient();
        // actionPlanner will give resource planner the actionSequence. 

        // Get action sequence 
        List<ActionModel> actionSequence = taskModel.ActionSequence;
        if (actionSequence == null || actionSequence.Count == 0)
            throw new ArgumentException("Action sequence cannot be empty");


        // iterate throught actions in actionSequence
        foreach (ActionModel action in actionSequence)
        {
            List<RedisInterface.RelationModel> imagesTmp = (await redisApiClient.ActionGetRelationByNameAsync(action.Id, "NEEDS")).ToList();
            List<RelationModel> images = new List<RelationModel>();
            foreach (RedisInterface.RelationModel imgTmp in imagesTmp)
            {
                RelationModel relation = _mapper.Map<RelationModel>(imgTmp);
                images.Add(relation);
            }

            // images -> list of all relations with images for the action
            foreach (RelationModel relation in images)
            {
                RedisInterface.InstanceModel instanceTmp = await redisApiClient.InstanceGetByIdAsync(relation.PointsTo.Id);
                //map
                InstanceModel instance = _mapper.Map<InstanceModel>(instanceTmp);

                if (CanBeReused(instance))
                {
                    var reusedInstance = await GetInstanceToReuse(instance, orchestratorApiClient);
                    if (reusedInstance is not null)
                        instance = reusedInstance;
                }
                // add instance to actions 
                action.Services.Add(instance);
            }
        }
        return taskModel;

    }

    private async Task<InstanceModel> GetInstanceToReuse(InstanceModel instance, Orchestrator.OrchestratorApiClient orchestratorApi)
    {
        try
        {
            var statuses = await orchestratorApi.NetAppStatusGetByInstanceIdAsync(instance.Id);
        }
        catch (Orchestrator.ApiException<ApiResponse> apiEx)
        {
            if (apiEx.StatusCode == (int) HttpStatusCode.NotFound)
            {
                _logger.LogDebug("No active instances for {name} with {id} were found", instance.Name, instance.Id);
                return null;
            }
            _logger.LogError(apiEx, "There was a problem while calling the Orchestrator");
        }
        catch (Exception ex)
        {
            _logger.LogError(ex, "Unable to retrieve the status of the specified NetApps from the orchestrator");
        }
        return null;
    }

    private bool CanBeReused(InstanceModel instance)
    {
        return instance.IsReusable != null && instance.IsReusable.Value;
    }
}