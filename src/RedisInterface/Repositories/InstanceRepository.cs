using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using System.Text.Json;

namespace Middleware.RedisInterface.Repositories
{
    public class InstanceRepository : BaseRepository<InstanceModel>, IInstanceRepository
    {
        public InstanceRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(RedisDbIndexEnum.Instance, redisClient, redisGraph)
        {
        }

        public async Task<InstanceModel> GetInstanceByIdAsync(Guid id)
        {
            string value = (string)await Db.JsonGetAsync(id.ToString());

            InstanceModel model = JsonSerializer.Deserialize<InstanceModel>(value);

            return model;
        }

        public Task<InstanceModel> PostInstanceAsync()
        {
            throw new NotImplementedException();
        }

        public Task<InstanceModel> PatchInstanceAsync()
        {
            throw new NotImplementedException();
        }

        public Task DeleteInstance(Guid id)
        {
            throw new NotImplementedException();
        }
    }
}
