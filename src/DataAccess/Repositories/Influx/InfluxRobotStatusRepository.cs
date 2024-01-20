using InfluxDB.Client;
using Middleware.DataAccess.Repositories.Abstract.Influx;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using ILogger = Serilog.ILogger;

namespace Middleware.DataAccess.Repositories.Influx;
public class InfluxRobotStatusRepository : InfluxRepository<RobotStatusModel, RobotStatusDto>, IInfluxRobotStatusRepository
{
    public InfluxRobotStatusRepository(IInfluxDBClient client, ILogger logger) : base(client, logger,bucket: "RobotStatus",objectType: "robot")
    {
    }
}
