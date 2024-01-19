using InfluxDB.Client.Api.Domain;
using InfluxDB.Client.Core.Flux.Domain;
using InfluxDB.Client.Writes;
using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "robotStatus-idx", StorageType = StorageType.Json, Prefixes = new[] { Prefix })]
public class RobotStatusDto : InfluxDto
{
    public const string Prefix = "RobotStatus";

    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;

    [Indexed]
    public string Name { get; set; } = default!;

    [Indexed]
    public string ActionSequenceId { get; set; } = default!;

    [Indexed]
    public int? CurrentlyExecutedActionIndex { get; set; }

    [Indexed]
    public int BatteryLevel { get; set; }

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

        var objActionSequenceId = String.Empty;
        Int32 objCurrentlyExecutedActionIndex = 0;
        Int32 objBatteryLevel = 0;


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
                    if (fieldNamee == "ActionSequenceId")
                    {
                        objActionSequenceId = valueString.ToString();
                    }
                    else if (fieldNamee == "CurrentlyExecutedActionIndex")
                    {
                        objCurrentlyExecutedActionIndex = Int32.Parse(valueString);
                    }
                    else if (fieldNamee == "BatteryLevel")
                    {
                        objBatteryLevel = Int32.Parse(valueString);
                    }
                }
            }
        }
        // assemble object from above structure
        return new RobotStatusDto {
            Id = objId,
            Name = objName,
            ActionSequenceId = objActionSequenceId,
            CurrentlyExecutedActionIndex = objCurrentlyExecutedActionIndex,
            BatteryLevel = objBatteryLevel,
            Timestamp = (DateTimeOffset)objTimestamp
        };
    }

    public override BaseModel FromInfluxDataToModel(List<FluxTable> fluxTables)
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

        var objActionSequenceId = String.Empty;
        Int32 objCurrentlyExecutedActionIndex = 0;
        Int32 objBatteryLevel = 0;


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
                    if (fieldNamee == "ActionSequenceId")
                    {
                        objActionSequenceId = valueString.ToString();
                    }
                    else if (fieldNamee == "CurrentlyExecutedActionIndex")
                    {
                        objCurrentlyExecutedActionIndex = Int32.Parse(valueString);
                    }
                    else if (fieldNamee == "BatteryLevel")
                    {
                        objBatteryLevel = Int32.Parse(valueString);
                    }
                }
            }
        }
        // assemble object from above structure
        return new RobotStatusModel
        {
            Id = Guid.Parse(objId),
            Name = objName,
            ActionSequenceId = Guid.Parse(objActionSequenceId),
            CurrentlyExecutedActionIndex = objCurrentlyExecutedActionIndex,
            BatteryLevel = objBatteryLevel,
            Timestamp = (DateTimeOffset)objTimestamp
        };
    }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new RobotStatusModel
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            Name = dto.Name,
            ActionSequenceId = Guid.Parse(dto.ActionSequenceId!),
            CurrentlyExecutedActionIndex = dto.CurrentlyExecutedActionIndex,
            BatteryLevel = dto.BatteryLevel,
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
            .Tag("robot", Name)
            .Field("ActionSequenceId", ActionSequenceId)
            .Field("CurrentlyExecutedActionIndex", CurrentlyExecutedActionIndex)
            .Field("BatteryLevel", BatteryLevel)
            .Timestamp(DateTime.UtcNow, WritePrecision.Ns);
        return point;
    }
}