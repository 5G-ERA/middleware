using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Neo4j.Driver;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using Serilog;

namespace Middleware.DataAccess.Repositories
{
    public class RedisUserRepository : RedisRepository<UserModel, UserDto>, IUserRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public RedisUserRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph, Microsoft.Extensions.Logging.ILogger<RedisUserRepository> logger, IDriver driver) : base(provider, redisGraph, true, logger, driver)
        {
        }
    }
}
