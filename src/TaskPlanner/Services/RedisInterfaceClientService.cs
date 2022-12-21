﻿using System.Web;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Models;
using Middleware.Common.Structs;
using Newtonsoft.Json;

namespace Middleware.TaskPlanner.Services
{
    public class RedisInterfaceClientService : IRedisInterfaceClientService
    {
        private readonly HttpClient _httpClient;
        private readonly ILogger<RedisInterfaceClientService> _logger;
        public RedisInterfaceClientService(IHttpClientFactory httpClientFactory, ILogger<RedisInterfaceClientService> logger)
        {
            _logger = logger;
            _httpClient = httpClientFactory.CreateClient(AppConfig.RedisApiClientName);
        }
        public async Task<bool> AddRelationAsync<TSource, TDirection>(TSource source, TDirection direction, string name)
            where TSource : BaseModel where TDirection : BaseModel
        {
            if (source is null) throw new ArgumentNullException(nameof(source));
            if (direction is null) throw new ArgumentNullException(nameof(direction));
            if (name is null) throw new ArgumentNullException(nameof(name));

            try
            {
                var relation = CreateRelation(source, direction, name);

                string url = $"/api/v1/{source.GetType().GetModelName()}/AddRelation";

                var result = await _httpClient.PostAsJsonAsync(url, JsonConvert.SerializeObject(relation));

                return result.IsSuccessStatusCode;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was an error while creating a relation");
                throw;
            }
        }

        private RelationModel CreateRelation<TSource, TDirection>(TSource source, TDirection direction, string name)
            where TSource : BaseModel
            where TDirection : BaseModel
        {
            var initiatesFrom = new GraphEntityModel(source.Id, source.Name, source.GetType());
            var pointsTo = new GraphEntityModel(direction.Id, direction.Name, direction.GetType());
            return new RelationModel(initiatesFrom, pointsTo, name);
        }

        public Task<ActionPlanModel> ActionPlanGetByIdAsync(Guid id)
        {
            return ActionPlanGetByIdAsync(id, CancellationToken.None);
        }
        public async Task<ActionPlanModel> ActionPlanGetByIdAsync(Guid id, CancellationToken token)
        {
            if (id == default)
                throw new ArgumentNullException(nameof(id));

            string url = $"/api/v1/Action/plan/{id}";
            try
            {
                var result = await _httpClient.GetAsync(url, token);

                if (result.IsSuccessStatusCode == false)
                {
                    throw new InvalidOperationException();
                }
                var body = await result.Content.ReadAsStringAsync(token);
                ActionPlanModel actionPlan = JsonConvert.DeserializeObject<ActionPlanModel>(body);
                return actionPlan;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was en error while calling for the action plan with id: {id}", id);
                throw;
            }
        }

        public async Task<RobotModel> RobotGetByIdAsync(Guid id)
        {
            return await RobotGetByIdAsync(id, CancellationToken.None);
        }

        public async Task<RobotModel> RobotGetByIdAsync(Guid robotId, CancellationToken token)
        {
            if (robotId == default)
                throw new ArgumentNullException(nameof(robotId));

            string url = $"/api/v1/robot/{robotId}";
            try
            {
                var result = await _httpClient.GetAsync(url, token);

                if (result.IsSuccessStatusCode == false)
                {
                    throw new InvalidOperationException();
                }
                var body = await result.Content.ReadAsStringAsync(token);
                RobotModel robot = JsonConvert.DeserializeObject<RobotModel>(body);
                return robot;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was en error while calling for the robot with robotId: {robotId}", robotId);
                throw;
            }
        }

        public async Task<TaskModel> TaskGetByIdAsync(Guid id)
        {
            return await TaskGetByIdAsync(id, CancellationToken.None);
        }
        public async Task<TaskModel> TaskGetByIdAsync(Guid taskId, CancellationToken token)
        {
            if (taskId == default)
                throw new ArgumentNullException(nameof(taskId));

            string url = $"/api/v1/task/{taskId}";
            try
            {
                var result = await _httpClient.GetAsync(url, token);

                if (result.IsSuccessStatusCode == false)
                {
                    throw new InvalidOperationException();
                }
                var body = await result.Content.ReadAsStringAsync(token);
                var task = JsonConvert.DeserializeObject<TaskModel>(body);
                return task;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was en error while calling for the task with taskId: {taskId}", taskId);
                throw;
            }
        }

        public async Task<InstanceModel> GetInstanceAlternativeAsync(Guid id)
        {
            return await GetInstanceAlternativeAsync(id, CancellationToken.None);
        }

        public async Task<InstanceModel> GetInstanceAlternativeAsync(Guid instanceId, CancellationToken token)
        {
            if (instanceId == default)
                throw new ArgumentNullException(nameof(instanceId));

            string url = $"/api/v1/Instance/alternative/{instanceId}";
            try
            {
                var result = await _httpClient.GetAsync(url, token);

                if (result.IsSuccessStatusCode == false)
                {
                    throw new InvalidOperationException();
                }
                var body = await result.Content.ReadAsStringAsync(token);
                var instance = JsonConvert.DeserializeObject<InstanceModel>(body);
                return instance;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was en error while calling for the instance alternative with instanceId: {instanceId}", instanceId);
                throw;
            }
        }

        public async Task<List<RelationModel>> GetRelationForActionAsync(Guid id, string relationName)
        {
            var action = new ActionModel { Id = id };
            return await GetRelationAsync(action, relationName);
        }

        public async Task<List<RelationModel>> GetRelationAsync<TSource>(TSource source, string relationName) where TSource : BaseModel
        {
            if (source is null)
                throw new ArgumentNullException(nameof(source));
            if (relationName is null)
                throw new ArgumentNullException(nameof(Relation));

            try
            {                
                var builder = new UriBuilder($"{_httpClient.BaseAddress.ToString().TrimEnd('/')}/api/v1/{source.GetType().GetModelName()}/relation/{relationName}");
                builder.Port = -1;
                var query = HttpUtility.ParseQueryString(builder.Query);
                query["id"] = $"{source.Id}";
                builder.Query = query.ToString();
                string url = builder.ToString();
                var result = await _httpClient.GetAsync(url);

                if (result.IsSuccessStatusCode == false)
                {
                    throw new InvalidOperationException();
                }
                var body = await result.Content.ReadAsStringAsync();
                var relations = JsonConvert.DeserializeObject<List<RelationModel>>(body);
                return relations;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was an error while creating a relation");
                throw;
            }
        }

        public async Task<ActionModel> ActionGetByIdAsync(Guid id)
        {
            return await ActionGetByIdAsync(id, CancellationToken.None);
        }

        public async Task<ActionModel> ActionGetByIdAsync(Guid id, CancellationToken token)
        {
            if (id == default)
                throw new ArgumentNullException(nameof(id));

            string url = $"/api/v1/action/{id}";
            try
            {
                var result = await _httpClient.GetAsync(url, token);

                if (result.IsSuccessStatusCode == false)
                {
                    throw new InvalidOperationException();
                }
                var body = await result.Content.ReadAsStringAsync(token);
                var action = JsonConvert.DeserializeObject<ActionModel>(body);
                return action;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was en error while calling for the action plan with id: {id}", id);
                throw;
            }
        }

        public async Task<ActionPlanModel> GetLatestActionPlanByRobotIdAsync(Guid robotId)
        {
            return await GetLatestActionPlanByRobotIdAsync(robotId, CancellationToken.None);
        }

        public async Task<ActionPlanModel> GetLatestActionPlanByRobotIdAsync(Guid robotId, CancellationToken token)
        {
            if (robotId == default)
                throw new ArgumentNullException(nameof(robotId));

            string url = $"/api/v1/action/plan/robot/{robotId}";
            try
            {
                var result = await _httpClient.GetAsync(url, token);

                if (result.IsSuccessStatusCode == false)
                {
                    throw new InvalidOperationException();
                }
                var body = await result.Content.ReadAsStringAsync(token);
                var actionPlan = JsonConvert.DeserializeObject<ActionPlanModel>(body);
                return actionPlan;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was en error while calling for the latest action plan with robotId: {robotId}", robotId);
                throw;
            }
        }

        public async Task<EdgeModel> GetEdgeByNameAsync(string name)
        {
            return await GetEdgeByNameAsync(name, CancellationToken.None);
        }

        public async Task<EdgeModel> GetEdgeByNameAsync(string name, CancellationToken token)
        {
            if (string.IsNullOrWhiteSpace(name))
                throw new ArgumentNullException(nameof(name));

            string url = $"/api/v1/edge/name/{name}";
            try
            {
                var result = await _httpClient.GetAsync(url, token);

                if (result.IsSuccessStatusCode == false)
                {
                    throw new InvalidOperationException();
                }
                var body = await result.Content.ReadAsStringAsync(token);
                var edge = JsonConvert.DeserializeObject<EdgeModel>(body);
                return edge;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was en error while calling for the edge with name: {name}", name);
                throw;
            }
        }

        public Task<CloudModel> GetCloudByNameAsync(string name)
        {
            return GetCloudByNameAsync(name, CancellationToken.None);
        }

        public async Task<CloudModel> GetCloudByNameAsync(string name, CancellationToken token)
        {
            if (string.IsNullOrWhiteSpace(name))
                throw new ArgumentNullException(nameof(name));

            string url = $"/api/v1/cloud/name/{name}";
            try
            {
                var result = await _httpClient.GetAsync(url, token);

                if (result.IsSuccessStatusCode == false)
                {
                    throw new InvalidOperationException();
                }
                var body = await result.Content.ReadAsStringAsync(token);
                var cloud = JsonConvert.DeserializeObject<CloudModel>(body);
                return cloud;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was en error while calling for the cloud with name: {name}", name);
                throw;
            }
        }
    }
}