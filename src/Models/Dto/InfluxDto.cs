using InfluxDB.Client.Core.Flux.Domain;
using InfluxDB.Client.Writes;

namespace Middleware.Models.Dto;
public abstract class InfluxDto : Dto
{
    public abstract PointData ToPointData();

    public abstract Dto FromInfluxDataDto(List<FluxTable> fluxTables);
}
