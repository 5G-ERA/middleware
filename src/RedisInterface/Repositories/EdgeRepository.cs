using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using Middleware.RedisInterface.Repositories.Abstract;
using NReJSON;
using RedisGraphDotNet.Client;
using StackExchange.Redis;
using System.Text.Json;

namespace Middleware.RedisInterface.Repositories
{
    public class EdgeRepository : BaseRepository<EdgeModel>, IEdgeRepository
    {
        public EdgeRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(RedisDbIndexEnum.Edges, redisClient, redisGraph)
        {
        }

        public async Task<EdgeModel> PatchEdgeAsync(Guid id, EdgeModel patch) 
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            EdgeModel currentModel = JsonSerializer.Deserialize<EdgeModel>(model);
            if (!string.IsNullOrEmpty(patch.EdgeStatus))
            {
                currentModel.EdgeStatus = patch.EdgeStatus;
            }
            if (patch.EdgeIp != null && Uri.IsWellFormedUriString(patch.EdgeIp.ToString(), UriKind.RelativeOrAbsolute))
            {
                currentModel.EdgeIp = patch.EdgeIp;
            }
            if (!string.IsNullOrEmpty(patch.MacAddress))
            {
                currentModel.MacAddress = patch.MacAddress;
            }
            if (!string.IsNullOrEmpty(patch.Cpu.ToString()))
            {
                currentModel.Cpu = patch.Cpu;
            }
            if (!string.IsNullOrEmpty(patch.Ram.ToString()))
            {
                currentModel.Ram = patch.Ram;
            }
            if (!string.IsNullOrEmpty(patch.VirtualRam.ToString()))
            {
                currentModel.VirtualRam = patch.VirtualRam;
            }
            if (!string.IsNullOrEmpty(patch.DiskStorage.ToString()))
            {
                currentModel.DiskStorage = patch.DiskStorage;
            }
            if (!string.IsNullOrEmpty(patch.NumberOfCores.ToString()))
            {
                currentModel.NumberOfCores = patch.NumberOfCores;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }
    }
}
