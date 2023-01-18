using Middleware.Models.Domain;
using Middleware.Models.Dto;
using Redis.OM;

namespace Middleware.DataAccess.Repositories.Redis;

public class RedisActionRepository : RedisRepository<ActionModel, ActionDto>
{
    public RedisActionRepository(RedisConnectionProvider provider) : base(provider)
    {
    }
}
