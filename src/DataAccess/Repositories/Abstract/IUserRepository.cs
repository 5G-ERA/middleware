using Middleware.Models.Domain;
using Middleware.Models.Dto;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IUserRepository : IRedisRepository<UserModel, UserDto>, IRelationRepository
    {
    }
}
