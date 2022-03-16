using Middleware.Common.Models;
using Middleware.RedisInterface.Enums;
using Middleware.RedisInterface.Repositories.Abstract;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.RedisInterface.Repositories
{
    public class ContainerImageRepository : BaseRepository<ContainerImageModel>, IContainerImageRepository
    {
        public ContainerImageRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph) : base(RedisDbIndexEnum.Containers, redisClient, redisGraph)
        {
        }

        public Task<ContainerImageModel> PatchContainerImageAsync(Guid id)
        {
            throw new NotImplementedException();
        }
    }
}
