using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface IRobotStatusRepository : IBaseRepository<RobotStatusModel>, IRelationRepository
{
    Task<RobotStatusModel> AddAsync(RobotStatusModel model, Func<Guid> guidProvider);

}