using System.Text.Json;
using Middleware.Common.Config;
using Middleware.Common.Models;

namespace Middleware.TaskPlanner.Services
{
    public class RedisInterfaceClientService
    {
        private readonly HttpClient _httpClient;
        private readonly ILogger<RedisInterfaceClientService> _logger;
        public RedisInterfaceClientService(IHttpClientFactory httpClientFactory, ILogger<RedisInterfaceClientService> logger)
        {
            _logger = logger;
            _httpClient = httpClientFactory.CreateClient(AppConfig.RedisApiClientName);
        }

        public Task<ActionPlanModel> ActionPlanGetByIdAsync(Guid id)
        {
            return ActionPlanGetByIdAsync(id, CancellationToken.None);
        }

        public async Task<bool> AddRelation<TSource, TDirection>(TSource source, TDirection direction, string name)
            where TSource : BaseModel where TDirection : BaseModel
        {
            if (source is null) throw new ArgumentNullException(nameof(source));
            if (direction is null) throw new ArgumentNullException(nameof(direction));
            if (name is null) throw new ArgumentNullException(nameof(name));

            try
            {
                var relation = CreateRelation(source, direction, name);

                string url = "api";
                    
                var result = await _httpClient.PostAsync(url, JsonSerializer.Serialize(relation));
            }
            catch (Exception ex)
            {
                
                throw;
            } 
            
            throw new NotImplementedException();
        }

        private RelationModel CreateRelation<TSource, TDirection>(TSource source, TDirection direction, string name) 
            where TSource : BaseModel 
            where TDirection : BaseModel
        {
            var initiatesFrom = new GraphEntityModel(source.Id, source.Name, source.GetType());
            var pointsTo = new GraphEntityModel(direction.Id, direction.Name, direction.GetType());
            return new RelationModel(initiatesFrom, pointsTo, name);
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
                ActionPlanModel actionPlan = JsonSerializer.Deserialize<ActionPlanModel>(body);
                return actionPlan;
            }
            catch (Exception ex)
            {
                _logger.LogError(ex, "There was en error while calling for the action plan with id: {id}", id);
                throw;
            }
        }
    }
}
