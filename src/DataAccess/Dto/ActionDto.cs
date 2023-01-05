using Middleware.DataAccess.Dto.Hardware;
using Redis.OM.Modeling;

namespace Middleware.DataAccess.Dto;

[Document(IndexName = "action-idx", StorageType = StorageType.Json, Prefixes =new[] { "Action" })]
internal class ActionDto
{
    [Indexed]
    public string? Id { get; set; }
    [Indexed]
    public string? Name { get; set; }
    [Indexed]
    public List<string> Tags { get; set; } = new();
    [Indexed]
    public string? ActionPriority { get; set; }

    public HardwareRequirements? HardwareRequirements { get; set; } = new();
    
}
