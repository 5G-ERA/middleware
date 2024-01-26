using Middleware.Models.Domain;
using Middleware.Models.Dto;

namespace Middleware.DataAccess.Repositories.Abstract.Influx;
public interface IInfluxNetAppStatusRepository: IInfluxRepository<NetAppStatusModel>
{
}
