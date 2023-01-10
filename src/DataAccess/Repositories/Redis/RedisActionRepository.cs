using Middleware.DataAccess.Dto;
using Middleware.DataAccess.Repositories.Abstract;
using Middleware.DataAccess.Mapping;
using Redis.OM;

namespace Middleware.DataAccess.Repositories.Redis;

public class RedisActionRepository : RedisRepository<ActionDto>
{
    public RedisActionRepository(RedisConnectionProvider provider) : base(provider)
    {
    }
}
