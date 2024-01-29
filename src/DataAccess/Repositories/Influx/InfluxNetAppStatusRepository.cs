using InfluxDB.Client;
using Middleware.DataAccess.Repositories.Abstract.Influx;
using Middleware.Models.Domain;
using Middleware.Models.Dto;
using ILogger = Serilog.ILogger;

namespace Middleware.DataAccess.Repositories.Influx;
public class InfluxNetAppStatusRepository : InfluxRepository<NetAppStatusModel, NetAppStatusDto>, IInfluxNetAppStatusRepository
{    
    public InfluxNetAppStatusRepository(IInfluxDBClient client, ILogger logger) : base(client, logger,bucket: NetAppStatusDto.Bucket, measurement: NetAppStatusDto.Measurement)
    {        
    }
}
