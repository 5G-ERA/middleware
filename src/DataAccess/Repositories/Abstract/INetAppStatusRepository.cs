using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface INetAppStatusRepository : IBaseRepository<NetAppStatusModel>, IRelationRepository
{
    Task<NetAppStatusModel> AddAsync(NetAppStatusModel model, Func<Guid> guidProvider);

}