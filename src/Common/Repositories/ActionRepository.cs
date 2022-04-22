using Middleware.Common.Models;
using Middleware.Common.Enums;
using Middleware.Common.Repositories.Abstract;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using System.Text.Json;
using Microsoft.Extensions.Logging;

namespace Middleware.Common.Repositories
{
    public class ActionRepository : BaseRepository<ActionModel>, IActionRepository
    {
        public ActionRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<ActionRepository> logger) : base(RedisDbIndexEnum.Action, redisClient, redisGraph, logger)
        {
        }


        public async Task<ActionModel> PatchActionAsync(Guid id, ActionModel patch)
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            ActionModel currentModel = JsonSerializer.Deserialize<ActionModel>(model);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Order.ToString()))
            {
                currentModel.Order = patch.Order;
            }
            if (!string.IsNullOrEmpty(patch.Placement))
            {
                currentModel.Placement = patch.Placement;
            }
            if (!string.IsNullOrEmpty(patch.ActionPriority))
            {
                currentModel.ActionPriority = patch.ActionPriority;
            }
            if (!string.IsNullOrEmpty(patch.Services.ToString()))
            {
                currentModel.Services = patch.Services;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }

    }
}
