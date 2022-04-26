using System.Text.Json;
using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using Middleware.Common.Repositories.Abstract;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.Common.Repositories
{
    public class CloudRepository : BaseRepository<CloudModel>, ICloudRepository
    {
        public CloudRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<CloudRepository> logger) : base(RedisDbIndexEnum.Cloud, redisClient, redisGraph, logger, true)
        {
        }

        public async Task<CloudModel> PatchCloudAsync(Guid id, CloudModel patch) 
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            CloudModel currentModel = JsonSerializer.Deserialize<CloudModel>(model);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.CloudStatus))
            {
                currentModel.CloudStatus = patch.CloudStatus;
            }
            if (patch.CloudIp != null && Uri.IsWellFormedUriString(patch.CloudIp.ToString(), UriKind.RelativeOrAbsolute))
            {
                currentModel.CloudIp = patch.CloudIp;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }
    }
}
