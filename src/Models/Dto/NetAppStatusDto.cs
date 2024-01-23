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
    public const string Measurement = "Heartbeat";
    private const string ObjectType = "Netapp";

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

    public override Dto? FromInfluxDataToDto(List<FluxTable> fluxTables)
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

        // static properties of all objects
        var objName = fluxTables[0].Records[0].GetValueByKey(ObjectType).ToString();
        var objId = fluxTables[0].Records[0].GetValueByKey("Id").ToString();
        var objTimestamp = fluxTables[0].Records[0].GetTimeInDateTime();

        // TODO: Guard extracted values
        if (objName == null || objId == null || objTimestamp == null)
        {
            return null;
        }

        Timestamp = (DateTimeOffset)objTimestamp;
        Id = objId;
        Name = objName;

        // dynamic properties of all objects
        // data of one parameter is kept in a table.Records as a table and could be accessed as table or by provided methods;
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
                        HardLimit = int.Parse(valueString);
                    }
                    else if (fieldNamee == "OptimalLimit")
                    {
                        OptimalLimit = int.Parse(valueString);
                    }
                    else if (fieldNamee == "CurrentRobotsCount")
                    {
                        CurrentRobotsCount = int.Parse(valueString);
                    }
                }
            }
        }
        return this;
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
        var point = PointData.Measurement(Measurement)
            .Tag("Id", Id)
            .Tag(ObjectType, Name)
            .Field("HardLimit", HardLimit)
            .Field("OptimalLimit", OptimalLimit)
            .Field("CurrentRobotsCount", CurrentRobotsCount)
            .Timestamp(DateTime.UtcNow, WritePrecision.Ns);
        return point;
    }
}
