using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "robotStatus-idx", StorageType = StorageType.Json, Prefixes = new[] { Prefix })]
public class RobotStatusDto : Dto
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
}