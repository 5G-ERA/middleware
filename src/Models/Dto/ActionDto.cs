using Middleware.Models.Domain;
using Middleware.Models.Dto.Hardware;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto;

[Document(IndexName = "action-idx", StorageType = StorageType.Json, Prefixes = new[] { "Action" })]
public class ActionDto : Dto
{
    [Indexed]
    [RedisIdField]
    public override string Id { get; set; } = default!;
    [Indexed]
    public string Name { get; init; } = default!;
    [Indexed]
    public List<string> Tags { get; init; } = new();
    [Indexed]
    public string ActionPriority { get; init; } = default!;

    public HardwareRequirements? HardwareRequirements { get; init; } = new();
    
    public override object ToModel()
    {
        return new ActionModel()
        {

        };
    }
}