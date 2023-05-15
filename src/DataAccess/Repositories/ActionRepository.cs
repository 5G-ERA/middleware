using System.Text.Json;
using Microsoft.Extensions.Logging;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Enums;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.DataAccess.Repositories
{
    public class ActionRepository : BaseRepository<ActionModel>, IActionRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public ActionRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<ActionRepository> logger) : base(RedisDbIndexEnum.Action, redisClient, redisGraph, logger, true)
        {
        }

        /// <summary>
        /// Patching properties for ActionModel
        /// </summary>
        /// <param name="id"></param>
        /// <param name="patch"></param>
        /// <returns> Patched model </returns>
        public async Task<ActionModel> PatchActionAsync(Guid id, ActionModel patch)
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            ActionModel currentModel = JsonSerializer.Deserialize<ActionModel>(model);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
            }

            if (!string.IsNullOrEmpty(patch.Order.ToString()))
            {
                currentModel.Order = patch.Order;
            }
            if (!string.IsNullOrEmpty(patch.Placement))
            {
                currentModel.Placement = patch.Placement;
            }
            if (!string.IsNullOrEmpty(patch.ActionPriority.ToString()))
            {
                currentModel.ActionPriority = patch.ActionPriority;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }
    }
}
