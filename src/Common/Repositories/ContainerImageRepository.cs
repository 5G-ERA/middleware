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
    public class ContainerImageRepository : BaseRepository<ContainerImageModel>, IContainerImageRepository
    {
        public ContainerImageRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<ContainerImageRepository> logger) : base(RedisDbIndexEnum.Container, redisClient, redisGraph, logger)
        {
        }

        public async Task<ContainerImageModel> PatchContainerImageAsync(Guid id, ContainerImageModel patch)
        {
            string model = (string)await Db.JsonGetAsync(id.ToString());
            ContainerImageModel currentModel = JsonSerializer.Deserialize<ContainerImageModel>(model);
            if (currentModel == null)
            {
                return null;
            }
            if (!string.IsNullOrEmpty(patch.Name))
            {
                currentModel.Name = patch.Name;
            }
            if (!string.IsNullOrEmpty(patch.Timestamp))
            {
                currentModel.Timestamp = patch.Timestamp;
            }
            if (!string.IsNullOrEmpty(patch.Description))
            {
                currentModel.Description = patch.Description;
            }
            await Db.JsonSetAsync(id.ToString(), JsonSerializer.Serialize(currentModel));
            return currentModel;
        }

    }
}
