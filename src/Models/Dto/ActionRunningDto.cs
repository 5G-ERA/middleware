using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "actionRunning-idx", StorageType = StorageType.Json, Prefixes = new[] { ActionRunningDto.Prefix })]
public class ActionRunningDto : Dto
{
    public const string Prefix = "ActionRunning";
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;
    [Indexed]
    public string ActionParentId { get; init; } = default!; // This is the ID in which the actionRunning us based from the normal Action
    [Indexed]
    public string ActionPlanId { get; init; } = default!;
    [Indexed]
    public string Name { get; init; } = default!;
    [Indexed]
    public List<string> Tags { get; init; } = new();
    [Indexed]
    public string ActionPriority { get; init; } = default!;

    public HardwareRequirements HardwareRequirements { get; init; } = new();

    public override BaseModel ToModel()
    {
        var dto = this;
        return new ActionRunningModel()
        {
            Id = Guid.Parse(dto.Id.Replace(Prefix, "")),
            ActionParentId = Guid.Parse(dto.ActionParentId.Replace(Prefix, "")),
            ActionPlanId = Guid.Parse(dto.ActionParentId.Replace(Prefix, "")),
            Name = dto.Name,
            Tags = dto.Tags,
            MinimumRam = dto.HardwareRequirements.MinimumRam,
            MinimumNumCores = dto.HardwareRequirements.MinimumNumCores,
            ActionPriority = dto.ActionPriority
        };
    }
}