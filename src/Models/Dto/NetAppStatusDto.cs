using InfluxDB.Client.Api.Domain;
using InfluxDB.Client.Core.Flux.Domain;
using InfluxDB.Client.Writes;
using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "netAppStatus-idx", StorageType = StorageType.Json, Prefixes = new[] { Prefix })]
public class NetAppStatusDto : InfluxDto
{
    public const string Prefix = "NetAppStatus";

    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;

    [Indexed]
    public string Name { get; set; } = default!;

    [Indexed]
    public int HardLimit { get; set; }

    [Indexed]
    public int OptimalLimit { get; set; }

    [Indexed]
    public int? CurrentRobotsCount { get; set; }

    [Indexed(Sortable = true)]
    public DateTimeOffset Timestamp { get; set; }

    public override Dto FromInfluxDataDto(List<FluxTable> fluxTables)
    {
        // Check if is not passed null value
        if (fluxTables == null || fluxTables.Count == 0)
        {
            return null;
        }
        if (fluxTables[0].Records.Count < 1)
        {
            return null;
        }

        Int32 objHardLimit = 0;
        Int32 objOptimalLimit = 0;
        Int32 objCurrentRobotsCount = 0;


        // static properties of all objects
        var objName = fluxTables[0].Records[0].GetValueByKey("robot").ToString();
        var objId = fluxTables[0].Records[0].GetValueByKey("id").ToString();
        var objTimestamp = fluxTables[0].Records[0].GetTimeInDateTime();

        // TODO: Guard extracted values
        if (objName == null || objId == null || objTimestamp == null)
        {
            return null;
        }

        // dynamic properties of all objects
        // data of one parameter is kept in a table.Records as a table and could be accessed as array or by provided methods;
        // one sample will have as many tables as many parameters the object has;
        foreach (var fluxTable in fluxTables)
        {
            foreach (var fluxrecord in fluxTable.Records)
            {
                var fieldNamee = fluxrecord.GetField().ToString();
                var extractedValue = fluxrecord.GetValue();

                if (extractedValue == null) return null;

                var valueString = extractedValue.ToString();
                if (valueString != null)
                {
                    if (fieldNamee == "HardLimit")
                    {
                        objHardLimit = Int32.Parse(valueString);
                    }
                    else if (fieldNamee == "OptimalLimit")
                    {
                        objOptimalLimit = Int32.Parse(valueString);
                    }
                    else if (fieldNamee == "CurrentRobotsCount")
                    {
                        objCurrentRobotsCount = Int32.Parse(valueString);
                    }
                }
            }
        }
        // assemble object from above structure
        return new NetAppStatusDto
        {
            Id = objId,
            Name = objName,
            HardLimit = objHardLimit,
            OptimalLimit = objOptimalLimit,
            CurrentRobotsCount = objCurrentRobotsCount,
            Timestamp = (DateTimeOffset)objTimestamp
        };
    }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new NetAppStatusModel
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            Name = dto.Name,
            HardLimit = dto.HardLimit,
            OptimalLimit = dto.OptimalLimit,
            CurrentRobotsCount = dto.CurrentRobotsCount,
            Timestamp = dto.Timestamp.DateTime
        };
    }

    public override PointData ToPointData()
    {
        var dto = this;
        var id = Guid.Parse(dto.Id!.Replace(Prefix, ""));
        var stringId = id.ToString();

        var point = PointData.Measurement("heartbeat")
            .Tag("id", stringId)
            .Tag("netapp", Name)
            .Field("HardLimit", HardLimit)
            .Field("OptimalLimit", OptimalLimit)
            .Field("CurrentRobotsCount", CurrentRobotsCount)
            .Timestamp(DateTime.UtcNow, WritePrecision.Ns);
        return point;
    }
}