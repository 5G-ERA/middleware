using InfluxDB.Client.Core.Flux.Domain;
using InfluxDB.Client.Writes;
using Middleware.Models.Domain;

namespace Middleware.Models.Dto;
public abstract class InfluxDto : Dto
{
    public abstract PointData ToPointData();

    public abstract Dto? FromInfluxDataToDto(List<FluxTable> fluxTables);
}
