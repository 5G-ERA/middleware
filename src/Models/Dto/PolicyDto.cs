using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "policy-idx", StorageType = StorageType.Json, Prefixes = new[] { "Policy" })]
public class PolicyDto: Dto
{
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; }
    [Indexed]
    public string? Name { get; set; }
    [Indexed]
    public string Type { get; set; }
    [Indexed(Sortable = true)]
    public DateTime? Timestamp { get; set; }
    [Indexed]
    public bool? IsActive { get; set; }
    [Indexed]
    public string Description { get; set; }
    [Indexed]
    public int IsExclusiveWithinType { get; set; }

    public override BaseModel ToModel()
    {
        var dto = this;
        return new PolicyModel()
        {
            Id = Guid.Parse(dto.Id!),
            Name = dto.Name,
            Type = dto.Type,
            Timestamp = dto.Timestamp,
            IsActive = dto.IsActive,
            Description = dto.Description,
            IsExclusiveWithinType = dto.IsExclusiveWithinType
        };
    }
}
