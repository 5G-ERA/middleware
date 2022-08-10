using System.Linq;
using System.Net;
using AutoMapper;
using Middleware.Common;
using Middleware.Common.Enums;
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

    private async Task<ActionModel> InferResource (ActionModel actionParam)
    {
        var redisApiClient = _apiClientBuilder.CreateRedisApiClient();
        List<Middleware.ResourcePlanner.RedisInterface.ActivePolicy> activePolicies = (await redisApiClient.PolicyGetActiveAsync()).ToList();
        Dictionary<Guid, List<Guid>> tempDic = new Dictionary<Guid, List<Guid>>();//Diccionary of Key policy ID, values List of results after query

        foreach (RedisInterface.ActivePolicy policy in activePolicies)
        {
            if (policy == null)
            {
                throw new ArgumentException("Index policy is empty"); //This should never happend
            }
            if (policy.PolicyName == "AllContainersInClosestMachine")
            {
                List<Guid> connectedEdges = (await redisApiClient.RobotGetConnectedEdgesIdsAsync(Guid.Empty)).ToList();
                List<Guid> freeEdges = (await redisApiClient.GetFreeEdgesIdsAsync(connectedEdges)).ToList();
                if (freeEdges.Count()==0)
                {
                    //if all of them are busy, check which one is less busy
                    List<Guid> lessBusyEdges = (await redisApiClient.GetLessBusyEdgesAsync(connectedEdges)).ToList();
                    tempDic.Add(policy.Id, lessBusyEdges);
                    
                }
                
            }//historical data second policy. --> succesful goal, less time used to perform action, more efficient,
             //which one did the task before? QoS & QoE. (Speed network, bandwidth, 5g or 4g, wifi, number of slices)
           
          
        }

        return actionParam;
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

                if (CanBeReused(instance) && taskModel.resourceLock==true)
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
            var statuses = (await orchestratorApi.NetAppStatusGetByInstanceIdAsync(instance.Id))?.ToList();
            if (statuses == null || statuses.Any() == false)
                return null;
            if (statuses.Count == 1)
            {
                var status = statuses.First();
                instance.ServiceInstanceId = status.Id;
                return instance;
            }

            var percentageUsage = statuses.Select(s =>
                    new
                    {
                        s.Id,
                        Percentage = s.CurrentRobotsCount / (decimal) s.HardLimit * 100,
                        // TODO: this will have to be expanded based on the resource prediction
                        // for example if the edge can accommodate another instance
                        Recommendation = s.CurrentRobotsCount < s.OptimalLimit ? NetAppStatus.Green :
                            s.CurrentRobotsCount >= s.HardLimit ? NetAppStatus.Red : NetAppStatus.Yellow
                    }).Where(s => s.Recommendation != NetAppStatus.Red)
                .OrderBy(s => s.Recommendation).ThenBy(s=>s.Percentage).ToList();

            // need for the deployment of the new service no to overload one in the red zone
            if (percentageUsage.Any() == false)
            {
                return null;
            }

            var instanceToReuse = percentageUsage.First();
            instance.ServiceInstanceId = instanceToReuse.Id;
            return instance;
        }
        catch (Orchestrator.ApiException<ApiResponse> apiEx)
        {
            if (apiEx.StatusCode == (int)HttpStatusCode.NotFound)
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