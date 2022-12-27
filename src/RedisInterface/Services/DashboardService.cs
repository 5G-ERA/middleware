using Microsoft.AspNetCore.Mvc.ApplicationModels;
using Middleware.Common;
using Middleware.Common.Enums;
using Middleware.Common.Helpers;
using Middleware.Common.Models;
using Middleware.Common.Repositories;
using Middleware.Common.Repositories.Abstract;
using Middleware.RedisInterface.Responses;
using System.Runtime.InteropServices;
using System.Threading.Tasks;

namespace Middleware.RedisInterface.Services
{
    public class DashboardService : IDashboardService
    {
        private readonly IRobotRepository _robotRepository;            
        private readonly ITaskRepository _taskRepository;
        private readonly IActionPlanRepository _actionPlanRepository;
        private readonly IEdgeRepository _edgeRepository;
        private readonly ICloudRepository _cloudRepository;
        private readonly IInstanceRepository _instanceRepository;


        public DashboardService(IRobotRepository robotRepository, 
            ITaskRepository taskRepository, 
            IActionPlanRepository actionPlanRepository, 
            IEdgeRepository edgeRepository, 
            ICloudRepository cloudRepository,
            IInstanceRepository instanceRepository)
        {
            _instanceRepository = instanceRepository;
            _cloudRepository = cloudRepository;
            _edgeRepository = edgeRepository;
            _actionPlanRepository = actionPlanRepository;
            _taskRepository = taskRepository;
            _robotRepository = robotRepository;
        }

        /// <summary>
        /// Basic control grid backend for netApps.
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
                                   netApp.ROSDistro,
                                   netApp.OnboardedTime);
                instanceResponse.Add(netAppTemp);
            }
            return new(filter.FilterResult(instanceResponse), instanceResponse.Count);
        }

        /// <summary>
        /// Basic control grid backend for edge and clouds
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
                    await _cloudRepository.GetRelation(cloud.Id,"LOCATED_AT",RelationDirection.Incoming);

                int noOfContainers = 0;
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

                int noOfContainers = 0;
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
            return new (filter.FilterResult(locations), locations.Count);
        }

        /// <summary>
        /// Basic control grid backend for robot-tasks
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
            return new (filter.FilterResult(responses), responses.Count);
        }

        /// <summary>
        /// Get all action sequences with only actions list names 
        /// </summary>
        /// <returns></returns>
        public async Task<List<actionSequenceResponse>> GetActionSequenceAsync()
        {
            var tasks = await _taskRepository.GetAllAsync();
            var responses = new List<actionSequenceResponse>();

            foreach (var tempTask in tasks)
            {
                List<string> tempNamesActions = new List<string>();
                List<Common.Models.ActionModel> actions = tempTask.ActionSequence;
                foreach (var action in actions) tempNamesActions.Add(action.Name);
                var response = new actionSequenceResponse(
                    tempTask.Name,
                    tempTask.Id,
                    tempNamesActions
                    );
                responses.Add(response);
            }
            return responses;
        }

        /// <summary>
        /// Gets a list of onboarding item types.
        /// </summary>
        /// <returns></returns>
        public async Task<List<string>> GetOnboardingItemNamesAsync()
        {
            List<string> tempOnboardItemTypes = new List<string>();
            var cloud = new CloudModel();
            var edge = new EdgeModel();
            var robot = new RobotModel();
            var instance = new InstanceModel();
            var action = new Middleware.Common.Models.ActionModel();
            var task = new Middleware.Common.Models.TaskModel();
            tempOnboardItemTypes.Add(cloud.GetType().ToString());
            tempOnboardItemTypes.Add(edge.GetType().ToString());
            tempOnboardItemTypes.Add(robot.GetType().ToString());
            tempOnboardItemTypes.Add(instance.GetType().ToString());
            tempOnboardItemTypes.Add(action.GetType().ToString());
            tempOnboardItemTypes.Add(task.GetType().ToString());

            return tempOnboardItemTypes;
        }

        /// <summary>
        /// Gets a list of robots with some of their data.
        /// </summary>
        /// <returns></returns>
        public async Task<Tuple<List<RobotResponse>, int>> GetRobotsDataAsync(PaginationFilter filter)
        {
            var robots = await _robotRepository.GetAllAsync();
            var robotsResponse = new List<RobotResponse>();
            foreach (var robot in robotsResponse)
            {
                var netAppTemp = new RobotResponse(robot.RobotId,
                robot.RobotName,
                                   robot.Status,
                                   robot.OnboardedTime,
                                   robot.ROSVersion,
                                   robot.ROSDistro,
                                   robot.Company);
                robotsResponse.Add(netAppTemp);
            }
            return new(filter.FilterResult(robotsResponse), robotsResponse.Count);
        }


    }
}
