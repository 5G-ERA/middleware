using Middleware.CentralApi.Sdk;
using Middleware.Common;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Domain.Slice;
using Middleware.Models.Enums;
using Middleware.Models.ExtensionMethods;
using Middleware.RedisInterface.Contracts.Mappings;
using Middleware.RedisInterface.Contracts.Responses;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Services;

public class DashboardService : IDashboardService
{
    private readonly IActionPlanRepository _actionPlanRepository;
    private readonly IActionRepository _actionRepository;
    private readonly ICentralApiClient _centralApiClient;
    private readonly ICloudRepository _cloudRepository;
    private readonly IEdgeRepository _edgeRepository;
    private readonly IInstanceRepository _instanceRepository;
    private readonly IRobotRepository _robotRepository;
    private readonly ISliceRepository _sliceRepository;
    private readonly ITaskRepository _taskRepository;


    public DashboardService(IRobotRepository robotRepository,
        ITaskRepository taskRepository,
        IActionPlanRepository actionPlanRepository,
        IEdgeRepository edgeRepository,
        ICloudRepository cloudRepository,
        IInstanceRepository instanceRepository,
        IActionRepository actionRepository,
        ICentralApiClient centralApiClient,
        ISliceRepository sliceRepository)
    {
        _instanceRepository = instanceRepository;
        _cloudRepository = cloudRepository;
        _edgeRepository = edgeRepository;
        _actionPlanRepository = actionPlanRepository;
        _taskRepository = taskRepository;
        _robotRepository = robotRepository;
        _actionRepository = actionRepository;
        _centralApiClient = centralApiClient;
        _sliceRepository = sliceRepository;
    }

    /// <summary>
    ///     Basic control grid backend for netApps.
    /// </summary>
    /// <param name="filter"></param>
    /// <returns></returns>
    public async Task<Tuple<List<NetAppsDetailsResponse>, int>> GetNetAppsDataListAsync(PaginationFilter filter)
    {
        var instances = await _instanceRepository.GetAllAsync();
        var instanceResponse = new List<NetAppsDetailsResponse>();
        foreach (var netApp in instances)
        {
            var netAppTemp = new NetAppsDetailsResponse(netApp.Name,
                netApp.InstanceFamily,
                netApp.RosVersion,
                netApp.RosDistro,
                netApp.OnboardedTime);
            instanceResponse.Add(netAppTemp);
        }

        return new(filter.FilterResult(instanceResponse), instanceResponse.Count);
    }

    /// <summary>
    ///     Basic control grid backend for edge and clouds
    /// </summary>
    /// <param name="filter"></param>
    /// <returns></returns>
    public async Task<Tuple<List<LocationStatusResponse>, int>> GetLocationsStatusListAsync(PaginationFilter filter)
    {
        var locations = new List<LocationStatusResponse>();
        var clouds = await _cloudRepository.GetAllAsync();

        foreach (var cloud in clouds)
        {
            var locatedInstances =
                await _cloudRepository.GetRelation(cloud.Id, "LOCATED_AT", RelationDirection.Incoming);

            var noOfContainers = 0;
            foreach (var item in locatedInstances)
            {
                var instanceContainers = await _instanceRepository.GetRelation(item.InitiatesFrom.Id, "NEEDS");
                noOfContainers += instanceContainers.Count;
            }

            var location = new LocationStatusResponse(cloud.Name,
                cloud.LastUpdatedTime,
                cloud.CloudStatus,
                cloud.IsOnline,
                noOfContainers > 0,
                noOfContainers);
            locations.Add(location);
        }

        var edges = await _edgeRepository.GetAllAsync();

        foreach (var edge in edges)
        {
            var locatedInstances =
                await _edgeRepository.GetRelation(edge.Id, "LOCATED_AT", RelationDirection.Incoming);

            var noOfContainers = 0;
            foreach (var item in locatedInstances)
            {
                var instanceContainers = await _instanceRepository.GetRelation(item.InitiatesFrom.Id, "NEEDS");
                noOfContainers += instanceContainers.Count;
            }

            var location = new LocationStatusResponse(edge.Name,
                edge.LastUpdatedTime,
                edge.EdgeStatus,
                edge.IsOnline,
                noOfContainers > 0,
                noOfContainers);
            locations.Add(location);
        }

        return new(filter.FilterResult(locations), locations.Count);
    }

    /// <summary>
    ///     Basic control grid backend for robot-tasks
    /// </summary>
    /// <param name="filter"></param>
    /// <returns></returns>
    public async Task<Tuple<List<TaskRobotResponse>, int>> GetRobotStatusListAsync(PaginationFilter filter)
    {
        var robots = await _robotRepository.GetAllAsync();
        var actionPlans = await _actionPlanRepository.GetAllAsync();
        var tasks = await _taskRepository.GetAllAsync();

        var responses = new List<TaskRobotResponse>();
        foreach (var ap in actionPlans)
        {
            var task = tasks.Where(t => t.Id == ap.TaskId).FirstOrDefault();
            var robot = robots.Where(r => r.Id == ap.RobotId).FirstOrDefault();
            if (task is null || robot is null)
                continue;

            var response = new TaskRobotResponse(
                ap.RobotId,
                robot.Name,
                task.Id,
                task.Name,
                ap.TaskStartedAt,
                ap.Status == "completed" ? ap.LastStatusChange : null,
                robot.RobotStatus);

            responses.Add(response);
        }

        return new(filter.FilterResult(responses), responses.Count);
    }

    /// <summary>
    ///     Get all action sequences with only actions list names
    /// </summary>
    /// <returns></returns>
    public async Task<List<ActionSequenceResponse>> GetActionSequenceAsync()
    {
        var tasks = await _taskRepository.GetAllAsync();
        var responses = new List<ActionSequenceResponse>();

        foreach (var taskTemp in tasks)
        {
            var tempNamesActions = new List<string>();
            var action = await _taskRepository.GetRelation(taskTemp.Id, "EXTENDS");
            foreach (var actionTemp in action)
            {
                var actionModelTemp = await _actionRepository.GetByIdAsync(actionTemp.PointsTo.Id);
                tempNamesActions.Add(actionModelTemp.Name);
            }

            var response = new ActionSequenceResponse(
                taskTemp.Name,
                taskTemp.Id,
                tempNamesActions
            );
            responses.Add(response);
            //tempNamesActions.Clear();
        }

        return responses;
    }

    /// <summary>
    ///     Gets a list of onboarding item types.
    /// </summary>
    /// <returns></returns>
    public List<string> GetOnboardingItemNames()
    {
        var types = new[]
        {
            typeof(CloudModel), typeof(EdgeModel), typeof(RobotModel), typeof(InstanceModel), typeof(TaskModel),
            typeof(ActionModel)
        };
        var typeNames = types.Select(t => t.Name.TrimSuffix("Model")).ToList();
        return typeNames;
    }

    /// <summary>
    ///     Gets a list of robots with some of their data.
    /// </summary>
    /// <returns></returns>
    public async Task<Tuple<List<DashboardRobotResponse>, int>> GetRobotsDataAsync(PaginationFilter filter)
    {
        var robots = await _robotRepository.GetAllAsync();
        var robotsResponse = new List<DashboardRobotResponse>();
        foreach (var robot in robots)
        {
            var netAppTemp = new DashboardRobotResponse(robot.Id,
                robot.Name,
                robot.RobotStatus,
                robot.OnboardedTime,
                robot.RosVersion,
                robot.RosDistro,
                robot.Manufacturer);
            robotsResponse.Add(netAppTemp);
        }

        return new(filter.FilterResult(robotsResponse), robotsResponse.Count);
    }

    /// <summary>
    ///     Get all relationModels possible to reconstruct the graph
    /// </summary>
    /// <param name="filter"></param>
    /// <returns></returns>
    public async Task<GraphResponse> GetAllRelationModelsAsync()
    {
        var resultSet = await _robotRepository.GetAllRelations();

        var entities = new List<GraphEntityModel>();
        var relations = new List<SimpleRelation>();
        foreach (var node in resultSet["n"])
        {
            var entity = new GraphEntityModel();
            if (node is not Node nd)
                continue;

            RedisValue? id = null, name = null, type = null;
            var props = nd.Properties;
            if (props.ContainsKey("ID"))
                id = props["ID"];
            if (props.ContainsKey("Type"))
                type = props["Type"];
            if (props.ContainsKey("Name"))
                name = props["Name"];

            if (id == null || (id != null && Guid.TryParse(id?.ToString(), out _)) == false)
                continue;

            entity.Id = Guid.Parse(id?.ToString());
            var typeStr = type?.ToString();
            entity.Type = (typeStr == "CONTAINER" ? "CONTAINERIMAGE" : typeStr)!;
            entity.Name = name?.ToString();

            entities.Add(entity);
        }

        var entityIds = entities.Select(e => e.Id.ToString()).ToList();

        for (var i = 0; i < resultSet["n"].Count; i++)
        {
            var initiatesId = (resultSet["n"][i] as Node).Properties["ID"].ToString();
            var type = resultSet["r"][i].GetType();
            var relationName = (resultSet["r"][i] as ScalarResult<string>).Value;

            if (relationName is null || entityIds.Contains(initiatesId) == false)
                continue;
            var pointsTo = (resultSet["m"][i] as Node)?.Properties["ID"].ToString();
            relations.Add(
                new()
                {
                    OriginatingId = Guid.Parse(initiatesId),
                    PointsToId = Guid.Parse(pointsTo),
                    RelationName = relationName
                });
        }

        var response = new GraphResponse { Entities = entities, Relations = relations };

        return response;
    }

    /// <inheritdoc />
    public async Task<IReadOnlyList<LocationResponse>> GetOrganizationStructureAsync(string orgName)
    {
        var locationsResponse = await _centralApiClient.GetAvailableLocations(orgName);
        if (locationsResponse is null) return null;

        var locations = new List<LocationResponse>();
        foreach (var loc in locationsResponse.Locations)
        {
            var slices = await GetSlicesForLocation(loc.Id, Enum.Parse<LocationType>(loc.Type));
            var slicesResp = slices.Select(s => s.ToSliceResponse()).ToArray();
            locations.Add(new()
            {
                Id = loc.Id,
                Name = loc.Name,
                Type = loc.Type,
                IsOnline = loc.IsOnline,
                IpAddress = loc.Address,
                Slices = slicesResp
            });
        }

        return locations;
    }

    private async Task<IReadOnlyList<SliceModel>> GetSlicesForLocation(Guid locId, LocationType locType)
    {
        var slices = new List<SliceModel>();
        var relations = locType == LocationType.Cloud
            ? await _cloudRepository.GetRelation(locId, "OFFERS")
            : await _edgeRepository.GetRelation(locId, "OFFERS");

        foreach (var relation in relations)
        {
            var slice = await _sliceRepository.GetByIdAsync(relation.PointsTo.Id);
            if (slice is not null) slices.Add(slice);
        }

        return slices;
    }
}