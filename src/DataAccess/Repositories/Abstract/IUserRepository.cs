using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract
{
    public interface IUserRepository : IBaseRepository<UserModel>, IRelationRepository
    {
    }
}
