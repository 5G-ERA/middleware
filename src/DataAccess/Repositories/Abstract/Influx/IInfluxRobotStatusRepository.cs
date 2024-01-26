using Middleware.Models.Domain;
using Middleware.Models.Dto;

namespace Middleware.DataAccess.Repositories.Abstract.Influx;
public interface IInfluxRobotStatusRepository : IInfluxRepository<RobotStatusModel>
{
}