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
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public CloudRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<CloudRepository> logger) : base(RedisDbIndexEnum.Cloud, redisClient, redisGraph, logger, true)
        {
        }

        /// <summary>
        /// Patching properties for CloudModel
        /// </summary>
        /// <param name="id"></param>
        /// <param name="patch"></param>
        /// <returns> Patched model </returns>
        public async Task<CloudModel> PatchCloudAsync(Guid id, CloudModel patch) 
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            CloudModel currentModel = JsonSerializer.Deserialize<CloudModel>(model);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
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

        public async Task<List<CloudModel>> GetCloudResourceDetailsbyNameAsync(string name)

        {
            List<CloudModel> cloudData = await ExecuteLuaQueryAsync("GetResourceCloudData " + name);
            return cloudData;
        }
        /// <summary>
        /// Retrieves active policies
        /// </summary>
        /// <returns> Active policies </returns>
        /// 
        public async Task<CloudModel> GetCloudResourceDetailsByNameAsync(string name)
        {
            //RedisValue[] testValues = new RedisValue[] { name };


            //object parameters = name;
            //List<EdgeModel> edgeData = await ExecuteLuaQueryAsync("GetResourceEdgeData", testValues);
            CloudModel cloud = (await GetAllAsync()).Where(x => x.Name == name).FirstOrDefault();
            return cloud;
            // return edgeData;
        }
    }
}
