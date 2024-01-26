using Middleware.Models.Domain;

namespace Middleware.DataAccess.Repositories.Abstract;

public interface IRobotStatusRepository : IBaseRepository<RobotStatusModel>, IRelationRepository
{
}