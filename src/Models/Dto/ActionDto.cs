using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "action-idx", StorageType = StorageType.Json, Prefixes = new[] { Prefix })]
public class ActionDto : Dto
{
    public const string Prefix = "Action";

    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;

    [Indexed]
    public string Name { get; init; } = default!;

    [Indexed]
    public List<string>? Tags { get; init; } = new();

    [Indexed]
    public string? ActionPriority { get; init; } = default!;

    [Indexed]
    public bool SingleNetAppEntryPoint { get; init; }

    public HardwareRequirements HardwareRequirements { get; init; } = new();

    public override BaseModel ToModel()
    {
        var dto = this;
        return new ActionModel
        {
            Id = Guid.Parse(dto.Id.Replace(Prefix, "")),
            Name = dto.Name,
            Tags = dto.Tags,
            MinimumRam = dto.HardwareRequirements.MinimumRam,
            MinimumNumCores = (int?)dto.HardwareRequirements.MinimumNumberOfCores,
            ActionPriority = dto.ActionPriority,
            SingleNetAppEntryPoint = dto.SingleNetAppEntryPoint
        };
    }
}