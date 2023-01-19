using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface INetAppStatusRepository : IBaseRepository<NetAppStatusModel>, IRelationRepository
{

}