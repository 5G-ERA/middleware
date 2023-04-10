using Microsoft.Extensions.Logging;
using Middleware.Models.Domain;
using Middleware.Models.ExtensionMethods;
using Middleware.RedisInterface.Contracts.Responses;
using Middleware.RedisInterface.Sdk.Client;

namespace Middleware.RedisInterface.Sdk
{
    public class RedisInterfaceClient : IRedisInterfaceClient
    {
        private readonly IRedisInterface _api;
        private readonly ILogger<RedisInterfaceClient> _logger;

        internal RedisInterfaceClient(IRedisInterface api, ILogger<RedisInterfaceClient> logger)
        {
            _api = api;
            _logger = logger;
        }

        public async Task<bool> AddRelationAsync<TSource, TDirection>(TSource source, TDirection direction, string name)
            where TSource : BaseModel where TDirection : BaseModel
        {
            if (source is null) throw new ArgumentNullException(nameof(source));
            if (direction is null) throw new ArgumentNullException(nameof(direction));
            if (name is null) throw new ArgumentNullException(nameof(name));

            var entity = source.GetType().GetModelName();
            var relation = CreateRelation(source, direction, name);
            await _api.RelationAdd(entity, relation);

            return true;
        }

        public async Task<bool> DeleteRelationAsync<TSource, TDirection>(TSource source, TDirection direction,
            string name) where TSource : BaseModel where TDirection : BaseModel
        {
            if (source is null) throw new ArgumentNullException(nameof(source));
            if (direction is null) throw new ArgumentNullException(nameof(direction));
            if (name is null) throw new ArgumentNullException(nameof(name));

            var entity = source.GetType().GetModelName();
            var relation = CreateRelation(source, direction, name);

            await _api.RelationDelete(entity, relation);
            return true;
        }

        private RelationModel CreateRelation<TSource, TDirection>(TSource source, TDirection direction, string name)
            where TSource : BaseModel
            where TDirection : BaseModel
        {
            var initiatesFrom = new GraphEntityModel(source.Id, source.Name, source.GetType());
            var pointsTo = new GraphEntityModel(direction.Id, direction.Name, direction.GetType());
            return new RelationModel(initiatesFrom, pointsTo, name);
        }

        public Task<ActionPlanModel?> ActionPlanGetByIdAsync(Guid id)
        {
            return ActionPlanGetByIdAsync(id, CancellationToken.None);
        }

        public async Task<ActionPlanModel?> ActionPlanGetByIdAsync(Guid id, CancellationToken token)
        {
            if (id == default)
                throw new ArgumentNullException(nameof(id));

            var result = await _api.ActionPlanGetById(id);

            return result.IsSuccessStatusCode ? result.Content : null;
        }

        public async Task<List<ActionPlanModel>?> ActionPlanGetAllAsync()
        {
            return await ActionPlanGetAllAsync(CancellationToken.None);
        }

        public async Task<List<ActionPlanModel>?> ActionPlanGetAllAsync(CancellationToken token)
        {
            var result = await _api.ActionPlanGetAllAsync();

            return result.IsSuccessStatusCode ? result.Content : null;
        }

        public async Task<bool> ActionPlanAddAsync(ActionPlanModel actionPlan)
        {
            return await ActionPlanAddAsync(actionPlan, CancellationToken.None);
        }

        public async Task<bool> ActionPlanAddAsync(ActionPlanModel actionPlan, CancellationToken token)
        {
            if (actionPlan is null)
                throw new ArgumentNullException(nameof(actionPlan));

            var result = await _api.ActionPlanAddAsync(actionPlan);

            return result.IsSuccessStatusCode;
        }

        public async Task<bool> ActionPlanDeleteAsync(Guid id)
        {
            return await ActionPlanDeleteAsync(id, CancellationToken.None);
        }

        public async Task<bool> ActionPlanDeleteAsync(Guid id, CancellationToken token)
        {
            if (id == default) throw new ArgumentNullException(nameof(id));

            await _api.ActionPlanDeleteAsync(id);

            return true;
        }

        public async Task<RobotResponse?> RobotGetByIdAsync(Guid id)
        {
            return await RobotGetByIdAsync(id, CancellationToken.None);
        }

        public async Task<RobotResponse?> RobotGetByIdAsync(Guid robotId, CancellationToken token)
        {
            if (robotId == default)
                throw new ArgumentNullException(nameof(robotId));

            var result = await _api.RobotGetById(robotId);

            return result.IsSuccessStatusCode ? result.Content : null;
        }


        public async Task<TaskResponse?> TaskGetByIdAsync(Guid id)
        {
            return await TaskGetByIdAsync(id, CancellationToken.None);
        }

        public async Task<TaskResponse?> TaskGetByIdAsync(Guid taskId, CancellationToken token)
        {
            if (taskId == default)
                throw new ArgumentNullException(nameof(taskId));

            var result = await _api.TaskGetById(taskId);

            return result.IsSuccessStatusCode ? result.Content : null;
        }

        public async Task<InstanceResponse?> GetInstanceAlternativeAsync(Guid id)
        {
            return await GetInstanceAlternativeAsync(id, CancellationToken.None);
        }

        public async Task<InstanceResponse?> GetInstanceAlternativeAsync(Guid instanceId, CancellationToken token)
        {
            if (instanceId == default)
                throw new ArgumentNullException(nameof(instanceId));

            var result = await _api.InstanceGetAlternative(instanceId);

            return result.IsSuccessStatusCode ? result.Content : null;
        }

        public async Task<List<RelationModel>?> GetRelationForActionAsync(Guid id, string relationName)
        {
            var action = new ActionModel { Id = id };
            return await GetRelationAsync(action, relationName);
        }

        public async Task<List<RelationModel>?> GetRelationAsync<TSource>(TSource source, string relationName)
            where TSource : BaseModel
        {
            if (source is null)
                throw new ArgumentNullException(nameof(source));
            if (relationName is null)
                throw new ArgumentNullException(nameof(relationName));


            var entity = source.GetType().GetModelName();
            var result = await _api.RelationGet(entity, relationName, source.Id);

            return result.IsSuccessStatusCode ? result.Content : null;
        }

        public async Task<InstanceResponse?> InstanceGetByIdAsync(Guid id)
        {
            return await InstanceGetByIdAsync(id, CancellationToken.None);
        }

        public async Task<InstanceResponse?> InstanceGetByIdAsync(Guid id, CancellationToken token)
        {
            if (id == default)
                throw new ArgumentNullException(nameof(id));

            var result = await _api.InstanceGetById(id);

            return result.IsSuccessStatusCode ? result.Content : null;
        }

        public async Task<GetContainersResponse?> ContainerImageGetForInstanceAsync(Guid id)
        {
            return await ContainerImageGetForInstanceAsync(id, CancellationToken.None);
        }

        public async Task<GetPoliciesResponse?> PolicyGetActiveAsync()
        {
            throw new NotImplementedException();
        }

        public async Task<GetCloudsResponse?> RobotGetConnectedCloudsAsync(Guid robotId)
        {
            throw new NotImplementedException();
        }

        public async Task<GetCloudsResponse?> GetFreeCloudIdsAsync(List<CloudModel> availableClouds)
        {
            throw new NotImplementedException();
        }

        public async Task<GetCloudsResponse?> GetLessBusyCloudsAsync(List<CloudModel> riconnectedClouds)
        {
            throw new NotImplementedException();
        }

        public async Task<CloudResponse?> CloudGetByNameAsync(string resourceName)
        {
            throw new NotImplementedException();
        }

        public async Task<GetEdgesResponse?> RobotGetConnectedEdgesIdsAsync(Guid robotId)
        {
            throw new NotImplementedException();
        }

        public async Task<GetEdgesResponse?> GetFreeEdgesIdsAsync(List<EdgeModel> riconnectedEdges)
        {
            throw new NotImplementedException();
        }

        public async Task<GetEdgesResponse?> GetLessBusyEdgesAsync(List<EdgeModel> riconnectedEdges)
        {
            throw new NotImplementedException();
        }

        public async Task<EdgeResponse> EdgeGetByNameAsync(string resourceName)
        {
            throw new NotImplementedException();
        }

        public async Task<GetContainersResponse?> ContainerImageGetForInstanceAsync(Guid id, CancellationToken token)
        {
            if (id == default)
                throw new ArgumentNullException(nameof(id));

            var result = await _api.ContainerImageGetForInstance(id);

            return result.IsSuccessStatusCode ? result.Content : null;
        }


        public async Task<ActionResponse?> ActionGetByIdAsync(Guid id)
        {
            return await ActionGetByIdAsync(id, CancellationToken.None);
        }

        public async Task<ActionResponse?> ActionGetByIdAsync(Guid id, CancellationToken token)
        {
            if (id == default)
                throw new ArgumentNullException(nameof(id));

            var result = await _api.ActionGetById(id);

            return result.IsSuccessStatusCode ? result.Content : null;
        }

        public async Task<ActionPlanModel?> GetLatestActionPlanByRobotIdAsync(Guid robotId)
        {
            return await GetLatestActionPlanByRobotIdAsync(robotId, CancellationToken.None);
        }

        public async Task<ActionPlanModel?> GetLatestActionPlanByRobotIdAsync(Guid robotId, CancellationToken token)
        {
            if (robotId == default)
                throw new ArgumentNullException(nameof(robotId));

            var result = await _api.ActionPlanGetLatestByRobotId(robotId);

            return result.IsSuccessStatusCode ? result.Content : null;
        }

        public async Task<EdgeResponse?> GetEdgeByNameAsync(string name)
        {
            return await GetEdgeByNameAsync(name, CancellationToken.None);
        }

        public async Task<EdgeResponse?> GetEdgeByNameAsync(string name, CancellationToken token)
        {
            if (string.IsNullOrWhiteSpace(name))
                throw new ArgumentNullException(nameof(name));

            var result = await _api.EdgeGetByName(name);

            return result.IsSuccessStatusCode ? result.Content : null;
        }

        public Task<CloudResponse?> GetCloudByNameAsync(string name)
        {
            return GetCloudByNameAsync(name, CancellationToken.None);
        }

        public async Task<CloudResponse?> GetCloudByNameAsync(string name, CancellationToken token)
        {
            if (string.IsNullOrWhiteSpace(name))
                throw new ArgumentNullException(nameof(name));

            var result = await _api.CloudGetByName(name);
            
            return result.IsSuccessStatusCode ? result.Content : null;
        }
    }
}