using Middleware.Models.Domain.Ros;

namespace Middleware.Models.Dto.Ros;

public class RosTopic
{
    public string Name { get; set; } = default!;
    public string? Type { get; set; }
    public string? Description { get; set; }
    public bool Enabled { get; set; }
    public string Compression { get; set; } = "none";

    public QosDto? Qos { get; set; }

    public RosTopicModel ToModel()
    {
        var dto = this;
        return new()
        {
            Name = dto.Name,
            Type = dto.Type,
            Description = dto.Description,
            Enabled = dto.Enabled,
            Compression = dto.Compression,
            Qos = dto.Qos?.ToModel()
        };
    }
}