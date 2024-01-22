using Middleware.Models.Domain.Ros;

namespace Middleware.Models.Dto.Ros;

public class RosService
{
    public string? Name { get; init; }
    public string? Description { get; init; }
    public string? Type { get; init; }

    public QosDto? Qos { get; init; }
    public RosServiceModel ToModel()
    {
        var dto = this;
        return new()
        {
            Name = dto.Name!,
            Description = dto.Description!,
            Type = dto.Type,
            Qos = dto.Qos?.ToModel()
        };
    }
}