using Middleware.Models.Domain;

namespace Middleware.Models.Dto.Ros;

public class Sensor
{
    public string Name { get; init; } = default!;
    public string Type { get; init; } = default!;
    public string? Description { get; init; }
    public List<string> Nodes { get; init; } = new();

    public int Number { get; set; }

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