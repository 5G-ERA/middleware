using Middleware.Models.Domain;
using Redis.OM.Modeling;

namespace Middleware.Models.Dto.Ros;

public class Sensor
{
    public string? Name { get; init; }
    public string? Type { get; init; }
    public string? Description { get; init; }

    [Indexed]
    public List<string> Nodes { get; init; } = new();

    public int Number { get; init; }

    public SensorModel ToModel()
    {
        var dto = this;
        return new SensorModel()
        {
            Name = dto.Name,
            Type = dto.Type,
            Description = dto.Description,
            Nodes = dto.Nodes,
        };
    }
}