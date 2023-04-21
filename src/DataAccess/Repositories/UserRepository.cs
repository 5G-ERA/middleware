using System.Linq.Expressions;
using Microsoft.Extensions.Logging;
using Middleware.Common.Enums;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Middleware.Models.Enums;
using Redis.OM.Searching;
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

        public Task DeleteAsync(UserModel model)
        {
            throw new NotImplementedException();
        }

        public Task<List<UserModel>> FindAsync(Expression<Func<UserDto, bool>> predicate)
        {
            throw new NotImplementedException();
        }

        public IRedisCollection<UserDto> FindQuery(Expression<Func<UserDto, bool>> predicate)
        {
            throw new NotImplementedException();
        }

        public Task<UserModel?> FindSingleAsync(Expression<Func<UserDto, bool>> predicate)
        {
            throw new NotImplementedException();
        }

        public Task UpdateAsync(UserModel model)
        {
            throw new NotImplementedException();
        }
    }
}
