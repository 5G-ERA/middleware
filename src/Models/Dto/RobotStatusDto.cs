using InfluxDB.Client.Api.Domain;
using InfluxDB.Client.Core.Flux.Domain;
using InfluxDB.Client.Writes;
using Middleware.Models.Domain;

namespace Middleware.Models.Dto;

public class RobotStatusDto : InfluxDto
{
    private const string Prefix = "RobotStatus";
    public const string Bucket = "RobotStatus";
    public const string Measurement = "Heartbeat";
    private const string ObjectType = "Robot";
    public override string Id { get; set; } = default!;

    public string Name { get; set; } = default!;

    public string? ActionSequenceId { get; set; }

    public int? CurrentlyExecutedActionIndex { get; set; }

    public int BatteryLevel { get; set; }

    public double CpuUtilisation { get; set; }

    public double RamUtilisation { get; set; }

    public DateTimeOffset Timestamp { get; set; }

    public override Dto? FromInfluxDataToDto(List<FluxTable> fluxTables)
    {
        if (fluxTables.Count == 0)
        {
            return null;
        }

        if (fluxTables[0].Records.Count < 0)
        {
            return null;
        }

        // static properties of all objects
        var objName = fluxTables[0].Records[0].GetValueByKey(ObjectType).ToString();
        var objId = fluxTables[0].Records[0].GetValueByKey("Id").ToString();
        var objTimestamp = fluxTables[0].Records[0].GetTimeInDateTime();

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
            foreach (var record in fluxTable.Records)
            {
                var fieldName = record.GetField();
                var extractedValue = record.GetValue();

                if (extractedValue == null || string.IsNullOrEmpty(fieldName)) continue;

                var valueString = extractedValue.ToString();
                if (valueString == null) continue;

                switch (fieldName)
                {
                    case "ActionSequenceId":
                        ActionSequenceId = valueString;
                        break;
                    case "CurrentlyExecutedActionIndex":
                        CurrentlyExecutedActionIndex = int.Parse(valueString);
                        break;
                    case "BatteryLevel":
                        BatteryLevel = int.Parse(valueString);
                        break;
                    case "CpuUtilisation":
                        CpuUtilisation = double.Parse(valueString);
                        break;
                    case "RamUtilisation":
                        RamUtilisation = double.Parse(valueString);
                        break;
                }
            }
        }
        return this;
    }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new RobotStatusModel
        {
            Id = Guid.Parse(dto.Id.Replace(Prefix, "")),
            Name = dto.Name,
            ActionSequenceId = Guid.Parse(dto.ActionSequenceId!),
            CurrentlyExecutedActionIndex = dto.CurrentlyExecutedActionIndex,
            BatteryLevel = dto.BatteryLevel,
            CpuUtilisation = dto.CpuUtilisation,
            RamUtilisation = dto.RamUtilisation,
            Timestamp = dto.Timestamp
        };
    }

    public override PointData ToPointData()
    {
        var point = PointData.Measurement(Measurement)
            .Tag("Id", Id)
            .Tag(ObjectType, Name)
            .Field("ActionSequenceId", ActionSequenceId)
            .Field("CurrentlyExecutedActionIndex", CurrentlyExecutedActionIndex)
            .Field("BatteryLevel", BatteryLevel)
            .Field("CpuUtilisation", CpuUtilisation)
            .Field("RamUtilisation", RamUtilisation)
            .Timestamp(DateTime.UtcNow, WritePrecision.Ns);
        return point;
    }
}