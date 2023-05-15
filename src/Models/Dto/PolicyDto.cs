using Middleware.Models.Domain;
using Middleware.Models.Enums;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "policy-idx", StorageType = StorageType.Json, Prefixes = new[] { PolicyDto.Prefix })]
public class PolicyDto : Dto
{
    public const string Prefix = "Policy";
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; }
    [Indexed]
    public string Name { get; set; } = default!;
    [Indexed]
    public string Type { get; set; } = default!;

    [Indexed]
    public string Scope { get; set; } = default!;

    [Indexed(Sortable = true)]
    public DateTimeOffset Timestamp { get; set; }
    [Indexed]
    public bool IsActive { get; set; }

    [Indexed]
    public string Description { get; set; } = default!;

    [Indexed]
    public int IsExclusiveWithinType { get; set; }

    public string Priority { get; init; } = default!;

    public override BaseModel ToModel()
    {
        var dto = this;
        return new PolicyModel
        {
            Id = Guid.Parse(dto.Id!.Replace(Prefix, "")),
            Name = dto.Name!,
            Type = Enum.Parse<PolicyType>(dto.Type),
            Scope = Enum.Parse<PolicyScope>(dto.Scope),
            Timestamp = dto.Timestamp.DateTime,
            IsActive = dto.IsActive,
            Description = dto.Description,
            IsExclusiveWithinType = dto.IsExclusiveWithinType,
            Priority = Enum.Parse<Priority>(dto.Priority)
        };
    }
}
