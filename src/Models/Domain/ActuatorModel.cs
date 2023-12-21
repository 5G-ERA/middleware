using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain;

public class ActuatorModel
{
    public string Name { get; set; } = default!;

    public string Type { get; set; } = default!;

    public int Number { get; set; }

    public List<string> Nodes { get; set; } = new();

    public Actuator ToDto()
    {
        return new()
        {
            Name = Name,
            Type = Type,
            Number = Number,
            Nodes = Nodes
        };
    }
}