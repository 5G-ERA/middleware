using Middleware.Models.Dto.Ros;

namespace Middleware.Models.Domain.Ros;

public class RosServiceModel
{
    public string Name { get; set; } = default!;
    public string Type { get; set; } = default!;
    public string? Description { get; set; }

    public Qos? Qos { get; set; }

    public RosService ToDto()
    {
        var domain = this;
        return new()
        {
            Description = domain.Description,
            Name = domain.Name
        };
    }
}