using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using Middleware.RedisInterface.Repositories.Abstract;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using System.Text.Json;

namespace Middleware.RedisInterface.Repositories
{
    public class CloudRepository : BaseRepository<CloudModel>, ICloudRepository
    {
        public CloudRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger logger) : base(RedisDbIndexEnum.Cloud, redisClient, redisGraph, logger)
        {
        }

        public async Task<CloudModel> PatchCloudAsync(Guid id, CloudModel patch) 
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            CloudModel currentModel = JsonSerializer.Deserialize<CloudModel>(model);
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
