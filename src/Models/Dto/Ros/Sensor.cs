using Middleware.Models.Domain;

namespace Middleware.Models.Dto.Ros;

public class Sensor
{
    public string? Name { get; set; }
    public string? Type { get; set; }
    public string? Description { get; set; }
    public List<string> Nodes { get; set; } = new();

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