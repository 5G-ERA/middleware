using Middleware.Models.Domain;
using Middleware.Models.Domain.ValueObjects;

namespace Middleware.Models.Dto.Ros;

public class RosTopic
{
    public string Name { get; set; } = default!;
    public string? Type { get; set; }
    public string? Description { get; set; }
    public bool Enabled { get; set; }

    public RosTopicModel ToModel()
    {
        var dto = this;
        return new()
        {
            Name = TopicName.From(dto.Name),
            Type = dto.Type,
            Description = dto.Description,
            Enabled = dto.Enabled
        };
    }
}