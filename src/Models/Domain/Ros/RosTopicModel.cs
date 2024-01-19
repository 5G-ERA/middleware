using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain.Ros;

public class RosTopicModel
{
    public string Name { get; init; } = default!;

    public string? Type { get; init; }

    public string? Description { get; init; }

    public string Compression { get; set; } = "none";

    public Qos? Qos { get; set; }

    public bool Enabled { get; set; }

    /// <summary>
    ///     Set the topic to be enabled.
    /// </summary>
    public void Enable()
    {
        Enabled = true;
    }

    /// <summary>
    ///     Set the topic to be disabled.
    /// </summary>
    public void Disable()
    {
        Enabled = false;
    }

    public RosTopic ToDto()
    {
        var domain = this;
        return new()
        {
            Name = domain.Name,
            Type = domain.Type,
            Description = domain.Description,
            Enabled = domain.Enabled,
            Compression = domain.Compression,
            Qos = domain.Qos?.ToDto()
        };
    }
}