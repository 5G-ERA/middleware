using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.Common.Models;
using Middleware.DataAccess.Repositories.Abstract;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

namespace Middleware.DataAccess.Repositories
{
    public class UserRepository : BaseRepository<UserModel>, IUserRepository
    {
        /// <summary>
        /// Default constructor
        /// </summary>
        /// <param name="redisClient"></param>
        /// <param name="redisGraph"></param>
        /// <param name="logger"></param>
        public UserRepository(IConnectionMultiplexer redisClient, IRedisGraphClient redisGraph, ILogger<UserRepository> logger) : base(RedisDbIndexEnum.User, redisClient, redisGraph, logger, false)
        {
        }
    }
}
