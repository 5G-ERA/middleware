using System.Text.Json;
using Middleware.Common.Config;
using Middleware.Models.Domain;

namespace Middleware.Orchestrator.Services
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
                var body = await result.Content.ReadAsStringAsync();
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
