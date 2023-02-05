using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Enums;
using Redis.OM.Contracts;
using RedisGraphDotNet.Client;
using StackExchange.Redis;

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
        public RedisUserRepository(IRedisConnectionProvider provider, IRedisGraphClient redisGraph) : base(provider, redisGraph, true)
        {
        }
    }
}
