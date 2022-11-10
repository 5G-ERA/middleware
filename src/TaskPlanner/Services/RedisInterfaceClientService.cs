using System.Text.Json;
using Middleware.Common.Config;
using Middleware.Common.ExtensionMethods;
using Middleware.Common.Models;
using Middleware.Common.Structs;

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
        public async Task<Either<bool, Exception>> AddRelation<TSource, TDirection>(TSource source, TDirection direction, string name)
            where TSource : BaseModel where TDirection : BaseModel
        {
            if (source is null) throw new ArgumentNullException(nameof(source));
            if (direction is null) throw new ArgumentNullException(nameof(direction));
            if (name is null) throw new ArgumentNullException(nameof(name));

            try
            {
                var relation = CreateRelation(source, direction, name);

                string url = $"/api/v1/{source.GetType().GetModelName()}/AddRelation";
                    
                var result = await _httpClient.PostAsJsonAsync(url, JsonSerializer.Serialize(relation));

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

        public Task<Either<ActionPlanModel, InvalidOperationException>> ActionPlanGetByIdAsync(Guid id)
        {
            return ActionPlanGetByIdAsync(id, CancellationToken.None);
        }
        public async Task<Either<ActionPlanModel, InvalidOperationException>> ActionPlanGetByIdAsync(Guid id, CancellationToken token)
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
                ActionPlanModel actionPlan = JsonSerializer.Deserialize<ActionPlanModel>(body);
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
        
        public async Task<RobotModel> RobotGetByIdAsync(Guid id, CancellationToken token)
        {
            if (id == default)
                throw new ArgumentNullException(nameof(id));

            string url = $"/api/v1/robot/{id}";
            try
            {
                var result = await _httpClient.GetAsync(url, token);

                if (result.IsSuccessStatusCode == false)
                {
                    throw new InvalidOperationException();
                }
                var body = await result.Content.ReadAsStringAsync(token);
                RobotModel robot = JsonSerializer.Deserialize<RobotModel>(body);
                return robot;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was en error while calling for the action plan with id: {id}", id);
                throw;
            }
        }

        public async Task<TaskModel> TaskGetByIdAsync(Guid id)
        {
            return await TaskGetByIdAsync(id, CancellationToken.None);
        }
        public async Task<TaskModel> TaskGetByIdAsync(Guid id, CancellationToken token)
        {
            if (id == default)
                throw new ArgumentNullException(nameof(id));

            string url = $"/api/v1/task/{id}";
            try
            {
                var result = await _httpClient.GetAsync(url, token);

                if (result.IsSuccessStatusCode == false)
                {
                    throw new InvalidOperationException();
                }
                var body = await result.Content.ReadAsStringAsync(token);
                var task = JsonSerializer.Deserialize<TaskModel>(body);
                return task;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was en error while calling for the action plan with id: {id}", id);
                throw;
            }
        }
    }
}
