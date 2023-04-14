using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Middleware.Models.Enums;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "actionRunning-idx", StorageType = StorageType.Json,
    Prefixes = new[] { ActionRunningDto.Prefix })]
public class ActionRunningDto : Dto
{
    public const string Prefix = "ActionRunning";
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;
    /// <summary>
    /// This is the ID in which the actionRunning is based from the normal Action
    /// </summary>
    [Indexed]
    public string ActionId { get; init; } = default!; 
    [Indexed]
    public string ActionPlanId { get; init; } = default!;
    [Indexed]
    public string Name { get; init; } = default!;
    [Indexed]
    public List<string>? Tags { get; init; } = new();
    [Indexed]
    public string? ActionPriority { get; init; }
    [Indexed]
    public string Placement { get; init; } = default!;
    [Indexed]
    public string PlacementType { get; init; } = default!;
    public HardwareRequirements HardwareRequirements { get; init; } = new();

    public override BaseModel ToModel()
    {
        var dto = this;
        return new ActionRunningModel()
        {
            Id = Guid.Parse(dto.Id),
            ActionId = Guid.Parse(dto.ActionId),
            ActionPlanId = Guid.Parse(dto.ActionPlanId),
            Name = dto.Name,
            Tags = dto.Tags,
            MinimumRam = dto.HardwareRequirements.MinimumRam,
            MinimumNumCores = dto.HardwareRequirements.MinimumNumCores,
            ActionPriority = dto.ActionPriority,
            Placement = dto.Placement,
            PlacementType = Enum.IsDefined(typeof(LocationType), PlacementType)
                ? Enum.Parse<LocationType>(dto.PlacementType)
                : LocationType.Unspecified
        };
    }
}